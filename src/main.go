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

// Track width tracking variables
var minTrackWidth int = 80  // Minimum expected track width
var maxTrackWidth int = 400 // Maximum expected track width
var widthConfidence int = 0 // How confident we are in the current range
var validWidths []int       // Store recent valid widths

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

// Update the track width range based on valid measurements
func updateTrackWidthRange(width int) {
	if width < 20 || width > 800 {
		return // Ignore obviously wrong values
	}

	// Store recent valid widths (keep last 20)
	validWidths = append(validWidths, width)
	if len(validWidths) > 20 {
		validWidths = validWidths[1:]
	}

	// Calculate average of recent widths
	if len(validWidths) > 5 {
		sum := 0
		for _, w := range validWidths {
			sum += w
		}
		avgWidth := sum / len(validWidths)

		// Set acceptable range: 60% to 140% of average
		minTrackWidth = avgWidth * 60 / 100
		maxTrackWidth = avgWidth * 140 / 100

		// Clamp to reasonable absolute limits
		if minTrackWidth < 40 {
			minTrackWidth = 40
		}
		if maxTrackWidth > 600 {
			maxTrackWidth = 600
		}

		widthConfidence++
		if widthConfidence > 10 {
			widthConfidence = 10
		}

		log.Debug().Int("width", width).Int("avg", avgWidth).Int("min", minTrackWidth).Int("max", maxTrackWidth).Msg("Track width range updated")
	}
}

// MODIFIED: Width-based filter to eliminate glare
func getValidTrackSlice(sliceDescriptors []SliceDescriptor, preferredX int) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	// First pass: Filter by width range (eliminates glare)
	filtered := []SliceDescriptor{}
	for _, desc := range sliceDescriptors {
		width := desc.End - desc.Start

		// Use dynamic width range if we have confidence, otherwise use default
		if widthConfidence > 3 {
			// We have learned the track width
			if width >= minTrackWidth && width <= maxTrackWidth {
				filtered = append(filtered, desc)
			} else {
				log.Debug().Int("width", width).Int("min", minTrackWidth).Int("max", maxTrackWidth).Msg("Filtered out by width")
			}
		} else {
			// Still learning - use generous defaults
			if width > 30 && width < 500 {
				filtered = append(filtered, desc)
			}
		}
	}

	if len(filtered) == 0 {
		return nil
	}

	// Try to find slice containing preferred X first
	for _, desc := range filtered {
		if preferredX >= desc.Start && preferredX <= desc.End {
			// Update track width range with this valid measurement
			updateTrackWidthRange(desc.End - desc.Start)
			return &desc
		}
	}

	// If none contains preferred X, find closest
	closest := &filtered[0]
	minDistance := abs(preferredX - (closest.Start+closest.End)/2)

	for i := 1; i < len(filtered); i++ {
		centerX := (filtered[i].Start + filtered[i].End) / 2
		distance := abs(preferredX - centerX)
		if distance < minDistance {
			minDistance = distance
			closest = &filtered[i]
		}
	}

	// Update track width range with this valid measurement
	updateTrackWidthRange(closest.End - closest.Start)
	return closest
}

// Detect if we're at an intersection (multiple track segments)
func isIntersection(sliceDescriptors []SliceDescriptor, minGap int) bool {
	if len(sliceDescriptors) >= 2 {
		for i := 0; i < len(sliceDescriptors)-1; i++ {
			gap := sliceDescriptors[i+1].Start - sliceDescriptors[i].End
			if gap > minGap {
				return true
			}
		}
	}
	return false
}

// Find the straight path (closest to image center) at intersections
func getStraightPath(sliceDescriptors []SliceDescriptor, imageWidth int) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	centerX := imageWidth / 2
	closest := &sliceDescriptors[0]
	minDistance := abs(centerX - (closest.Start+closest.End)/2)

	for i := 1; i < len(sliceDescriptors); i++ {
		segmentCenter := (sliceDescriptors[i].Start + sliceDescriptors[i].End) / 2
		distance := abs(centerX - segmentCenter)
		if distance < minDistance {
			minDistance = distance
			closest = &sliceDescriptors[i]
		}
	}

	return closest
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
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
		currentWidth := buf.Cols()
		currentHeight := buf.Rows()

		log.Debug().Int("width", currentWidth).Int("height", currentHeight).Msg("Read image")

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
			gocv.Threshold(buf, &buf, float32(thresholdValue), 255.0, gocv.ThresholdBinary+gocv.ThresholdOtsu)
			kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(5, 5))
			gocv.Dilate(buf, &buf, kernel)
			gocv.Erode(buf, &buf, kernel)
			kernel.Close()
		}

		var longestConsecutive *SliceDescriptor = nil
		var foundSliceY int = sliceY

		newBarY := verticalScanUp(&buf, preferredX, currentHeight-10) + 2
		if newBarY >= currentHeight {
			newBarY = currentHeight - 1
		}

		usedSlice := uint32(newBarY)
		if usedSlice < uint32(sliceY) {
			usedSlice = uint32(sliceY)
		}

		// Multi-scan for track detection
		for uint32(usedSlice) < (uint32(currentHeight)-1) && (longestConsecutive == nil) {
			usedSlice += 10

			if int(usedSlice) >= currentHeight-1 {
				break
			}

			horizontalSlice := buf.Region(image.Rect(0, int(usedSlice), currentWidth, int(usedSlice)+1))
			sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)

			// Check for intersection first
			if isIntersection(sliceDescriptors, 30) {
				log.Debug().Msg("Intersection detected - taking straight path")
				longestConsecutive = getStraightPath(sliceDescriptors, currentWidth)
				if longestConsecutive != nil {
					foundSliceY = int(usedSlice)
				}
			} else {
				// Use width-based filter to eliminate glare
				longestConsecutive = getValidTrackSlice(sliceDescriptors, preferredX)
				if longestConsecutive != nil {
					foundSliceY = int(usedSlice)
				}
			}

			if longestConsecutive != nil && (preferredX < longestConsecutive.Start || preferredX > longestConsecutive.End) {
				longestConsecutive = nil
			}
			horizontalSlice.Close()
		}

		// Smooth steering to reduce wobble
		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2
			// Smoothing: 70% old, 30% new
			preferredX = (preferredX*7 + middleX*3) / 10
		}

		canvasObjects := make([]*pb_output.CanvasObject, 0)
		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(longestConsecutive.Start),
							Y: uint32(foundSliceY),
						},
						Radius: 2,
					},
				},
			})
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(longestConsecutive.End),
							Y: uint32(foundSliceY),
						},
						Radius: 2,
					},
				},
			})
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(middleX),
							Y: uint32(foundSliceY),
						},
						Radius: 2,
					},
				},
			})
		}

		canvas := pb_output.Canvas{
			Objects: canvasObjects,
			Width:   uint32(currentWidth),
			Height:  uint32(currentHeight),
		}

		compressionParams := []int{gocv.IMWriteJpegQuality, 30}
		imgBytes, err := gocv.IMEncodeWithParams(".jpg", buf, compressionParams)
		if err != nil {
			log.Err(err).Msg("Error encoding image")
			return err
		}

		horizontalScans := make([]*pb_output.HorizontalScan, 0)
		if longestConsecutive != nil {
			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  uint32(longestConsecutive.Start),
				XRight: uint32(longestConsecutive.End),
				Y:      uint32(foundSliceY),
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
						Width:  uint32(currentWidth),
						Height: uint32(currentHeight),
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
