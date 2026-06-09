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
				if currentConsecutive.End-currentConsecutive.Start > 5 { // Minimum 5px to avoid noise
					res = append(res, *currentConsecutive)
				}
				currentConsecutive = nil
			}
		}
	}

	if currentConsecutive != nil && currentConsecutive.End-currentConsecutive.Start > 5 {
		res = append(res, *currentConsecutive)
	}

	return res
}

// Improved glare handling with adaptive filtering
func getLongestConsecutiveWhiteSlice(sliceDescriptors []SliceDescriptor, preferredX int, isCloseToCar bool) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	// Adaptive filtering based on distance from car
	minWidth := 15
	maxWidth := 600

	if isCloseToCar {
		// Closer to car, track appears wider
		minWidth = 30
		maxWidth = 800
	}

	// Filter out glare (too wide) and noise (too narrow)
	filtered := []SliceDescriptor{}
	for _, desc := range sliceDescriptors {
		width := desc.End - desc.Start
		if width > minWidth && width < maxWidth {
			filtered = append(filtered, desc)
		}
	}

	if len(filtered) == 0 {
		return nil
	}

	// First try to find slice containing preferred X
	for _, desc := range filtered {
		if preferredX >= desc.Start && preferredX <= desc.End {
			log.Debug().Int("preferredX", preferredX).Int("start", desc.Start).Int("end", desc.End).Msg("Found slice with preferred X")
			return &desc
		}
	}

	// If none contains preferred X, find closest to preferred X
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

	log.Debug().Int("preferredX", preferredX).Int("selectedCenter", (closest.Start+closest.End)/2).Msg("Selected closest slice")
	return closest
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

// Check for finish line (two black lines in the middle)
func checkFinishLine(img *gocv.Mat, y int) bool {
	if img.Empty() {
		return false
	}

	width := img.Cols()
	height := img.Rows()

	if y >= height {
		return false
	}

	// Scan a horizontal line at given Y
	row := img.Row(y)
	defer row.Close()

	// Look for two distinct black regions in the middle third of the image
	middleStart := width / 3
	middleEnd := 2 * width / 3

	blackRegions := 0
	inBlackRegion := false

	for x := middleStart; x < middleEnd; x++ {
		pixel := row.GetUCharAt(0, x)
		if pixel == 0 { // Black pixel
			if !inBlackRegion {
				blackRegions++
				inBlackRegion = true
			}
		} else {
			inBlackRegion = false
		}
	}

	// Finish line detected if we have 2 distinct black regions (the two black lines)
	return blackRegions >= 2
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

	// Track following variables
	lastValidMiddleX := preferredX
	consecutiveLostFrames := 0
	maxLostFrames := 10

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
			// Convert to grayscale
			gocv.CvtColor(buf, &buf, gocv.ColorBGRToGray)
			// Apply adaptive threshold for better glare handling
			gocv.AdaptiveThreshold(buf, &buf, 255, gocv.AdaptiveThresholdMean, gocv.ThresholdBinary, 51, 10)

			// Morphological operations to clean up
			kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(3, 3))
			gocv.MorphologyEx(buf, &buf, gocv.MorphClose, kernel)
			gocv.Erode(buf, &buf, kernel)
			gocv.Dilate(buf, &buf, kernel)
			kernel.Close()
		}

		// Check for finish line first
		finishLineDetected := false
		finishY := currentHeight - 20 // Check near bottom of image

		// Scan multiple Y positions for finish line
		for yOffset := 0; yOffset < 40; yOffset += 10 {
			if checkFinishLine(&buf, finishY-yOffset) {
				finishLineDetected = true
				break
			}
		}

		var longestConsecutive *SliceDescriptor = nil
		var foundSliceY int = sliceY

		// Start scanning from bottom up
		startY := currentHeight - 20
		scanStep := 5 // Smaller steps for better detection

		// Multi-scan with fallback
		for y := startY; y > sliceY && longestConsecutive == nil; y -= scanStep {
			if y >= currentHeight {
				continue
			}

			horizontalSlice := buf.Region(image.Rect(0, y, currentWidth, y+1))
			sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)

			// Check if we're close to the car (bottom of image)
			isCloseToCar := y > currentHeight-50
			longestConsecutive = getLongestConsecutiveWhiteSlice(sliceDescriptors, preferredX, isCloseToCar)

			if longestConsecutive != nil {
				foundSliceY = y
			}
			horizontalSlice.Close()
		}

		// If no track found in first pass, try wider scan
		if longestConsecutive == nil {
			consecutiveLostFrames++
			log.Warn().Int("consecutiveLost", consecutiveLostFrames).Msg("No track detected")

			// Use last known good position with gradual recovery
			if consecutiveLostFrames < maxLostFrames {
				// Use last valid middle X
				preferredX = lastValidMiddleX
			} else {
				// Completely lost - reset to center
				preferredX = currentWidth / 2
				consecutiveLostFrames = maxLostFrames // Stay at center until track found
			}
		} else {
			consecutiveLostFrames = 0
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2

			// Smooth the steering (avoid jerky movements)
			preferredX = (preferredX*3 + middleX) / 4
			lastValidMiddleX = preferredX
		}

		// Prepare canvas for debugging
		canvasObjects := make([]*pb_output.CanvasObject, 0)

		if longestConsecutive != nil {
			middleX := (longestConsecutive.Start + longestConsecutive.End) / 2

			// Draw track boundaries
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(longestConsecutive.Start),
							Y: uint32(foundSliceY),
						},
						Radius: 3,
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
						Radius: 3,
					},
				},
			})
			// Draw center line
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Line_{
					Line: &pb_output.CanvasObject_Line{
						Start: &pb_output.CanvasObject_Point{
							X: uint32(middleX),
							Y: uint32(foundSliceY),
						},
						End: &pb_output.CanvasObject_Point{
							X: uint32(middleX),
							Y: uint32(currentHeight),
						},
					},
				},
			})
		}

		// Add finish line indication to canvas if detected
		if finishLineDetected {
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Text_{
					Text: &pb_output.CanvasObject_Text{
						Content: "FINISH LINE",
						Position: &pb_output.CanvasObject_Point{
							X: 10,
							Y: 30,
						},
					},
				},
			})
		}

		canvas := pb_output.Canvas{
			Objects: canvasObjects,
			Width:   uint32(currentWidth),
			Height:  uint32(currentHeight),
		}

		// Encode image with lower quality for bandwidth
		compressionParams := []int{gocv.IMWriteJpegQuality, 30}
		imgBytes, err := gocv.IMEncodeWithParams(".jpg", buf, compressionParams)
		if err != nil {
			log.Err(err).Msg("Error encoding image")
			return err
		}

		horizontalScans := make([]*pb_output.HorizontalScan, 0)

		if finishLineDetected {
			// Special scan to indicate finish line (Y=9999 as expected by logger)
			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  0,
				XRight: uint32(currentWidth),
				Y:      9999, // Special value for finish line detection
			})
			log.Info().Msg("FINISH LINE DETECTED - Sending to logger")
		} else if longestConsecutive != nil {
			// Normal track scan
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

		log.Debug().Bool("finishLine", finishLineDetected).Int("preferredX", preferredX).Msg("Sent image")
	}
}

func onTerminate(sig os.Signal) error {
	log.Info().Msg("Terminating")
	return nil
}

func main() {
	roverlib.Run(run, onTerminate)
}
