package main

import (
	"fmt"
	"image"
	"os"
	"time"

	pb_output "github.com/VU-ASE/rovercom/v2/packages/go/outputs"
	roverlib "github.com/VU-ASE/roverlib-go/v2/src"
	"gocv.io/x/gocv"
	"google.golang.org/protobuf/proto"

	"github.com/rs/zerolog/log"
)

type SliceDescriptor struct {
	Start int
	End   int
}

var thresholdValue int

func verticalScanUp(image *gocv.Mat, x int, startY int) int {
	y := startY
	for y >= 0 {
		if image.GetUCharAt(y, x) == 0 {
			return y
		}
		y--
	}
	return y + 1
}

func getConsecutiveWhitePointsFromSlice(imageSlice *gocv.Mat) []SliceDescriptor {
	res := []SliceDescriptor{}
	var currentConsecutive *SliceDescriptor = nil

	for i := 0; i < imageSlice.Cols()-1; i++ {
		currentByte := imageSlice.GetVecbAt(0, i)[0]

		if currentByte != byte(0) {
			if currentConsecutive == nil {
				currentConsecutive = &SliceDescriptor{Start: i, End: i}
			} else {
				currentConsecutive.End = i
			}
		} else {
			if currentConsecutive != nil {
				if currentConsecutive.End-currentConsecutive.Start > 0 {
					res = append(res, *currentConsecutive)
				}
				currentConsecutive = nil
			}
		}
	}

	if currentConsecutive != nil && currentConsecutive.End-currentConsecutive.Start > 0 {
		res = append(res, *currentConsecutive)
	}

	return res
}

// Width-based selection, calibrated from logged data:
//
//	< 20px              -> noise, reject
//	20-500px            -> normal track candidate
//	500px - 95% width   -> glare merged with track, reject
//	>= 95% width        -> figure-8 crossing / washout, accept (steer straight)
func getLongestConsecutiveWhiteSlice(sliceDescriptors []SliceDescriptor, preferredX int, imgWidth int) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	fullWidthMin := int(float64(imgWidth) * 0.95)

	filtered := []SliceDescriptor{}
	for _, desc := range sliceDescriptors {
		width := desc.End - desc.Start
		if width >= fullWidthMin {
			filtered = append(filtered, desc)
		} else if width > 20 && width < 500 {
			filtered = append(filtered, desc)
		}
	}

	if len(filtered) == 0 {
		return nil
	}

	longest := filtered[0]
	for _, desc := range filtered {
		if preferredX > desc.Start && preferredX < desc.End {
			log.Debug().Int("preferredX", preferredX).Msg("Returned slice containing preferred X")
			return &desc
		}
		if (desc.End - desc.Start) > (longest.End - longest.Start) {
			longest = desc
		}
	}

	return &longest
}

// detectFinishLine looks for the finish strip: two black bars side by side
// INSIDE the white track. It scans the rows just below the row where the
// track was found, restricted to the track's x-span (shrunk to exclude the
// black borders). A row containing >= 2 black runs of >= 25px each is a
// finish-line row. Requiring 2+ such rows debounces noise.
//
// Nothing else produces this signature: corners are one white run, the
// crossing is full white, glare is white, floor noise is outside the span.
func detectFinishLine(buf *gocv.Mat, track *SliceDescriptor, foundSliceY int, imgHeight int) bool {
	if track == nil {
		return false
	}

	xL := track.Start + 30
	xR := track.End - 30
	if xR-xL < 100 {
		return false
	}

	rowsWithBars := 0

	for y := foundSliceY + 3; y <= foundSliceY+70 && y < imgHeight-1; y += 6 {
		blackRuns := 0
		runLen := 0

		for x := xL; x <= xR; x++ {
			if buf.GetUCharAt(y, x) < 40 {
				runLen++
			} else {
				if runLen >= 20 {
					blackRuns++
				}
				runLen = 0
			}
		}

		if runLen >= 20 {
			blackRuns++
		}

		if blackRuns >= 2 {
			rowsWithBars++
		}
	}

	return rowsWithBars >= 2
}
func run(service roverlib.Service, configuration *roverlib.ServiceConfiguration) error {
	if configuration == nil {
		return fmt.Errorf("configuration cannot be accessed")
	}

	gstPipeline, err := configuration.GetStringSafe("gstreamer-pipeline")
	if err != nil {
		log.Err(err).Msg("Failed to get gstreamer-pipeline from tuning")
		return err
	}
	thFloat, err := configuration.GetFloatSafe("threshold-value")
	if err != nil {
		return err
	}
	thresholdValue = int(thFloat)

	imgWidthFloat, err := configuration.GetFloatSafe("img-width")
	if err != nil {
		return err
	}
	imgWidth := int(imgWidthFloat)

	imgHeightFloat, err := configuration.GetFloatSafe("img-height")
	if err != nil {
		return err
	}
	imgHeight := int(imgHeightFloat)

	imgFpsFloat, err := configuration.GetFloatSafe("img-fps")
	if err != nil {
		return err
	}
	imgFps := int(imgFpsFloat)

	gstPipeline = fmt.Sprintf(gstPipeline, imgWidth, imgHeight, imgFps)
	log.Info().Str("pipeline", gstPipeline).Msg("Using gstreamer pipeline")

	imageOutput := service.GetWriteStream("path")
	if imageOutput == nil {
		return fmt.Errorf("failed to get write stream 'path'")
	}

	cam, err := gocv.OpenVideoCapture(gstPipeline)
	if err != nil {
		return err
	}
	defer cam.Close()

	buf := gocv.NewMat()
	defer buf.Close()

	sliceY := int(imgHeightFloat * 0.60)
	preferredX := imgWidth / 2

	for {
		if ok := cam.Read(&buf); !ok {
			log.Warn().Msg("Error reading from camera")
			continue
		}
		if buf.Empty() {
			continue
		}
		imgWidth := buf.Cols()
		imgHeight := buf.Rows()

		newThreshold, err := configuration.GetFloat("threshold-value")
		if err != nil {
			log.Err(err).Msg("Failed to get threshold value from tuning")
			continue
		} else if thresholdValue != int(newThreshold) {
			log.Info().Float64("threshold", newThreshold).Msg("Got new threshold value")
			thresholdValue = int(newThreshold)
		}

		// Dual threshold mode: 1 = Otsu auto, >1 = fixed manual, 0 = off
		if thresholdValue > 0 {
			gocv.CvtColor(buf, &buf, gocv.ColorBGRToGray)
			if thresholdValue == 1 {
				gocv.Threshold(buf, &buf, 0, 255.0, gocv.ThresholdBinary+gocv.ThresholdOtsu)
			} else {
				gocv.Threshold(buf, &buf, float32(thresholdValue), 255.0, gocv.ThresholdBinary)
			}
			kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
			gocv.Dilate(buf, &buf, kernel)
			gocv.Erode(buf, &buf, kernel)
			kernel.Close()
		}

		var longestConsecutive *SliceDescriptor = nil
		var foundSliceY int = sliceY

		newBarY := verticalScanUp(&buf, preferredX, imgHeight-10) + 2
		if newBarY >= imgHeight {
			newBarY = imgHeight - 1
		}

		usedSlice := uint32(newBarY)
		if usedSlice < uint32(sliceY) {
			usedSlice = uint32(sliceY)
		}

		for uint32(usedSlice) < (uint32(imgHeight)-1) && (longestConsecutive == nil) {
			usedSlice += 10

			if int(usedSlice) >= imgHeight-1 {
				break
			}

			horizontalSlice := buf.Region(image.Rect(0, int(usedSlice), imgWidth, int(usedSlice)+1))
			sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)
			longestConsecutive = getLongestConsecutiveWhiteSlice(sliceDescriptors, preferredX, imgWidth)

			if longestConsecutive != nil && (preferredX < longestConsecutive.Start || preferredX > longestConsecutive.End) {
				longestConsecutive = nil
			} else if longestConsecutive != nil {
				foundSliceY = int(usedSlice)
			}
			horizontalSlice.Close()
		}

		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2
			preferredX = middleX
		} else {
			preferredX = preferredX + (imgWidth/2-preferredX)/10
		}

		// Finish line check: two black bars inside the track below the scan row
		finishLineDetected := detectFinishLine(&buf, longestConsecutive, foundSliceY, imgHeight)
		if finishLineDetected {
			log.Info().Msg("FINISH LINE DETECTED")
		}

		canvasObjects := make([]*pb_output.CanvasObject, 0)
		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{X: uint32(longestConsecutive.Start), Y: uint32(foundSliceY)},
						Radius: 1,
					},
				},
			})
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{X: uint32(longestConsecutive.End), Y: uint32(foundSliceY)},
						Radius: 1,
					},
				},
			})
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{X: uint32(middleX), Y: uint32(foundSliceY)},
						Radius: 1,
					},
				},
			})
		}

		canvas := pb_output.Canvas{
			Objects: canvasObjects,
			Width:   uint32(imgWidth),
			Height:  uint32(imgHeight),
		}

		var compressionParams [2]int
		compressionParams[0] = gocv.IMWriteJpegQuality
		compressionParams[1] = 30
		imgBytes, err := gocv.IMEncodeWithParams(".jpg", buf, compressionParams[:])
		if err != nil {
			log.Err(err).Msg("Error encoding image")
			return err
		}

		horizontalScans := make([]*pb_output.HorizontalScan, 0)
		if longestConsecutive != nil {
			scanY := uint32(foundSliceY)
			// Y=9999 marks the finish line for the logger.
			// XLeft/XRight stay REAL so the controller keeps steering normally
			// (the controller only reads XLeft and XRight).
			if finishLineDetected {
				scanY = 9999
			}
			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  uint32(longestConsecutive.Start),
				XRight: uint32(longestConsecutive.End),
				Y:      scanY,
			})
		} else {
			log.Debug().Msg("No trajectory added")
		}

		output := pb_output.SensorOutput{
			SensorId:  25,
			Timestamp: uint64(time.Now().UnixMilli()),
			SensorOutput: &pb_output.SensorOutput_CameraOutput{
				CameraOutput: &pb_output.CameraSensorOutput{
					Resolution: &pb_output.Resolution{
						Width:  uint32(imgWidth),
						Height: uint32(imgHeight),
					},
					DebugFrame: &pb_output.DebugFrame{
						Jpeg:   imgBytes.GetBytes(),
						Canvas: &canvas,
					},
					HorizontalScans: horizontalScans,
				},
			},
		}
		outputBytes, err := proto.Marshal(&output)
		imgBytes.Close()

		if err != nil {
			log.Err(err).Msg("Error marshalling sensor output")
			continue
		}

		err = imageOutput.WriteBytes(outputBytes)
		if err != nil {
			log.Err(err).Int("byte len", len(outputBytes)).Msg("Error sending image")
			return err
		}
	}
}

func onTerminate(sig os.Signal) error {
	log.Info().Msg("Terminating")
	return nil
}

func main() {
	roverlib.Run(run, onTerminate)
}
