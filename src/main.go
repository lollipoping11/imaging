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

func getLongestConsecutiveWhiteSlice(sliceDescriptors []SliceDescriptor, preferredX int) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	// Filter out slices that are too wide (glare) or too narrow (noise)
	filtered := []SliceDescriptor{}
	for _, desc := range sliceDescriptors {
		width := desc.End - desc.Start
		if width > 20 && width < 500 {
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

var thresholdValue int

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
	if err != nil {
		return err
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
			log.Warn().Err(err).Msg("Error reading from camera")
			continue
		}
		if buf.Empty() {
			continue
		}
		imgWidth := buf.Cols()
		imgHeight := buf.Rows()

		log.Debug().Int("width", imgWidth).Int("height", imgHeight).Msg("Read image")

		newThreshold, err := configuration.GetFloat("threshold-value")
		if err != nil {
			log.Err(err).Msg("Failed to get threshold value from tuning")
			continue
		} else if thresholdValue != int(newThreshold) {
			log.Info().Float64("threshold", newThreshold).Msg("Got new threshold value")
			thresholdValue = int(newThreshold)
		}

		if thresholdValue > 0 {
			// Convert to grayscale
			gocv.CvtColor(buf, &buf, gocv.ColorBGRToGray)
			// Use Otsu thresholding — glare rejection handled by width filter
			gocv.Threshold(buf, &buf, float32(thresholdValue), 255.0, gocv.ThresholdBinary+gocv.ThresholdOtsu)
			// Clean up noise
			kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
			gocv.Dilate(buf, &buf, kernel)
			gocv.Erode(buf, &buf, kernel)
		}

		var longestConsecutive *SliceDescriptor = nil

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

			horizontalSlice := buf.Region(image.Rect(0, sliceY, imgWidth, sliceY+1))
			sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)
			longestConsecutive = getLongestConsecutiveWhiteSlice(sliceDescriptors, preferredX)

			if longestConsecutive != nil && (preferredX < longestConsecutive.Start || preferredX > longestConsecutive.End) {
				longestConsecutive = nil
			}
			horizontalSlice.Close()
		}

		canvasObjects := make([]*pb_output.CanvasObject, 0)
		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2
			preferredX = middleX

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{X: uint32(longestConsecutive.Start), Y: uint32(sliceY)},
						Radius: 1,
					},
				},
			})
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{X: uint32(longestConsecutive.End), Y: uint32(sliceY)},
						Radius: 1,
					},
				},
			})
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{X: uint32(middleX), Y: uint32(sliceY)},
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

		horizontal_scans := make([]*pb_output.HorizontalScan, 0)
		if longestConsecutive != nil {
			horizontal_scans = append(horizontal_scans, &pb_output.HorizontalScan{
				XLeft:  uint32(longestConsecutive.Start),
				XRight: uint32(longestConsecutive.End),
				Y:      uint32(sliceY),
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
					HorizontalScans: horizontal_scans,
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

		log.Debug().Msg("Sent image")
	}
}

func onTerminate(sig os.Signal) error {
	log.Info().Msg("Terminating")
	return nil
}

func main() {
	roverlib.Run(run, onTerminate)
}
