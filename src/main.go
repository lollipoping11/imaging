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

type TrackScan struct {
	Start int
	End   int
	Y     int
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
				currentConsecutive = &SliceDescriptor{
					Start: i,
					End:   i,
				}
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

	longest := sliceDescriptors[0]

	for _, desc := range sliceDescriptors {
		if preferredX > desc.Start && preferredX < desc.End {
			return &desc
		}

		if (desc.End - desc.Start) > (longest.End - longest.Start) {
			longest = desc
		}
	}

	return &longest
}

func getTrackScanAtY(buf *gocv.Mat, y int, preferredX int, previousWidth float64) *TrackScan {
	imgWidth := buf.Cols()
	imgHeight := buf.Rows()

	if y < 0 || y >= imgHeight-1 {
		return nil
	}

	horizontalSlice := buf.Region(image.Rect(0, y, imgWidth, y+1))
	defer horizontalSlice.Close()

	sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)
	candidate := getLongestConsecutiveWhiteSlice(sliceDescriptors, preferredX)

	if candidate == nil {
		return nil
	}

	width := candidate.End - candidate.Start

	// Hard limits:
	// Too small = noise/glare patch.
	// Too large = glare/washed-out image.
	if width < 320 {
		return nil
	}

	if width > int(float64(imgWidth)*0.90) {
		return nil
	}

	// Adaptive limits:
	// Generous so corners are not skipped.
	if previousWidth > 0 {
		minAllowed := previousWidth * 0.75
		maxAllowed := previousWidth * 1.25

		if float64(width) < minAllowed || float64(width) > maxAllowed {
			return nil
		}
	}

	return &TrackScan{
		Start: candidate.Start,
		End:   candidate.End,
		Y:     y,
	}
}

func findStableTrackScan(buf *gocv.Mat, sliceY int, preferredX int, previousWidth float64) *TrackScan {
	imgHeight := buf.Rows()

	// First try the exact same scan line as normal imaging.
	normalScan := getTrackScanAtY(buf, sliceY, preferredX, previousWidth)
	if normalScan != nil {
		return normalScan
	}

	// Only if normal scan looks bad, try nearby backup lines.
	// Closest first to keep driving behavior close to base imaging.
	offsets := []int{-15, 15, -30, 30}

	for _, offset := range offsets {
		y := sliceY + offset

		if y < int(float64(imgHeight)*0.35) || y > int(float64(imgHeight)*0.80) {
			continue
		}

		scan := getTrackScanAtY(buf, y, preferredX, previousWidth)
		if scan != nil {
			return scan
		}
	}

	return nil
}

func rowBlackRatioInTrack(buf *gocv.Mat, y int, xLeft int, xRight int) float64 {
	if y < 0 || y >= buf.Rows() {
		return 0.0
	}

	if xLeft < 0 {
		xLeft = 0
	}

	if xRight >= buf.Cols() {
		xRight = buf.Cols() - 1
	}

	if xRight <= xLeft {
		return 0.0
	}

	blackCount := 0
	total := xRight - xLeft

	for x := xLeft; x < xRight; x++ {
		if buf.GetUCharAt(y, x) == 0 {
			blackCount++
		}
	}

	return float64(blackCount) / float64(total)
}

func detectFinishLineInTrack(buf *gocv.Mat, scan TrackScan) bool {
	imgHeight := buf.Rows()

	// Shrink edges slightly so black borders do not count.
	xLeft := scan.Start + 20
	xRight := scan.End - 20

	if xRight <= xLeft {
		return false
	}

	startY := int(float64(imgHeight) * 0.20)
	endY := int(float64(imgHeight) * 0.78)

	bandsFound := 0
	inBand := false
	bandHeight := 0
	lastBandEnd := -9999

	for y := startY; y < endY; y++ {
		ratio := rowBlackRatioInTrack(buf, y, xLeft, xRight)

		// Finish line = black row across a big part of the white track.
		isBlackRow := ratio > 0.38

		if isBlackRow {
			if !inBand {
				inBand = true
				bandHeight = 1
			} else {
				bandHeight++
			}
		} else {
			if inBand {
				if bandHeight >= 2 && bandHeight <= 35 {
					gap := y - lastBandEnd

					if gap > 3 {
						bandsFound++
						lastBandEnd = y
					}
				}

				inBand = false
				bandHeight = 0
			}
		}
	}

	if inBand && bandHeight >= 2 && bandHeight <= 35 {
		bandsFound++
	}

	return bandsFound >= 2
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

	cam, err := gocv.OpenVideoCapture(gstPipeline)
	if err != nil {
		return err
	}
	defer cam.Close()

	buf := gocv.NewMat()
	defer buf.Close()

	sliceY := int(imgHeightFloat * 0.60)
	preferredX := imgWidth / 2

	// Based on your normal logs: usually around 420-540.
	previousWidth := 490.0

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

		if thresholdValue > 0 {
			gocv.CvtColor(buf, &buf, gocv.ColorBGRToGray)

			gocv.Threshold(
				buf,
				&buf,
				float32(thresholdValue),
				255.0,
				gocv.ThresholdBinary+gocv.ThresholdOtsu,
			)

			kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
			gocv.Dilate(buf, &buf, kernel)
			gocv.Erode(buf, &buf, kernel)
			kernel.Close()
		}

		trackScan := findStableTrackScan(&buf, sliceY, preferredX, previousWidth)

		finishLineDetected := false

		if trackScan != nil {
			width := trackScan.End - trackScan.Start
			middleX := (trackScan.Start + trackScan.End) / 2

			// Smooth center update so one bad frame does not yank steering.
			preferredX = int(0.75*float64(preferredX) + 0.25*float64(middleX))

			// Smooth width too.
			previousWidth = 0.80*previousWidth + 0.20*float64(width)

			finishLineDetected = detectFinishLineInTrack(&buf, *trackScan)

			if finishLineDetected {
				log.Info().
					Int("xLeft", trackScan.Start).
					Int("xRight", trackScan.End).
					Int("width", width).
					Msg("FINISH LINE DETECTED")
			}
		}

		canvasObjects := make([]*pb_output.CanvasObject, 0)

		if trackScan != nil {
			middleX := (trackScan.Start + trackScan.End) / 2

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(trackScan.Start),
							Y: uint32(trackScan.Y),
						},
						Radius: 1,
					},
				},
			})

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(trackScan.End),
							Y: uint32(trackScan.Y),
						},
						Radius: 1,
					},
				},
			})

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(middleX),
							Y: uint32(trackScan.Y),
						},
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

		if trackScan != nil {
			scanY := uint32(trackScan.Y)

			// Keep XLeft/XRight normal.
			// Only Y=9999 marks finish line for the logger.
			if finishLineDetected {
				scanY = 9999
			}

			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  uint32(trackScan.Start),
				XRight: uint32(trackScan.End),
				Y:      scanY,
			})
		} else {
			log.Debug().Msg("No stable track scan found")
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
