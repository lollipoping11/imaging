package main

import (
	"fmt"
	"image"
	"math"
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

// PID Controller for smooth steering
type PIDController struct {
	Kp float64 // Proportional gain
	Ki float64 // Integral gain
	Kd float64 // Derivative gain

	integral   float64
	lastError  float64
	lastOutput float64
}

func NewPIDController(kp, ki, kd float64) *PIDController {
	return &PIDController{
		Kp: kp,
		Ki: ki,
		Kd: kd,
	}
}

func (pid *PIDController) Update(error float64, dt float64) float64 {
	// Proportional term
	proportional := pid.Kp * error

	// Integral term with anti-windup
	pid.integral += error * dt
	// Clamp integral to prevent windup
	if pid.integral > 100 {
		pid.integral = 100
	} else if pid.integral < -100 {
		pid.integral = -100
	}
	integral := pid.Ki * pid.integral

	// Derivative term
	derivative := pid.Kd * (error - pid.lastError) / dt
	pid.lastError = error

	// Calculate output
	output := proportional + integral + derivative

	// Smooth with previous output (low-pass filter)
	output = output*0.7 + pid.lastOutput*0.3
	pid.lastOutput = output

	return output
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
				if currentConsecutive.End-currentConsecutive.Start > 5 {
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

func getBestTrackSlice(sliceDescriptors []SliceDescriptor, preferredX int, trackWidth int) *SliceDescriptor {
	if len(sliceDescriptors) == 0 {
		return nil
	}

	// Expected track width (can be adjusted based on camera height)
	expectedWidth := trackWidth
	minWidth := expectedWidth * 60 / 100  // 60% of expected
	maxWidth := expectedWidth * 140 / 100 // 140% of expected

	// Filter by realistic track width
	filtered := []SliceDescriptor{}
	for _, desc := range sliceDescriptors {
		width := desc.End - desc.Start
		if width > minWidth && width < maxWidth {
			filtered = append(filtered, desc)
		}
	}

	if len(filtered) == 0 {
		// Fallback: accept any reasonable width
		for _, desc := range sliceDescriptors {
			width := desc.End - desc.Start
			if width > 20 && width < 800 {
				filtered = append(filtered, desc)
			}
		}
	}

	if len(filtered) == 0 {
		return nil
	}

	// Find slice closest to preferred X (weighted by size)
	best := &filtered[0]
	bestScore := float64(-1)

	for i := range filtered {
		centerX := (filtered[i].Start + filtered[i].End) / 2
		distance := float64(abs(preferredX - centerX))
		width := filtered[i].End - filtered[i].Start

		// Score: prioritize closeness, but also prefer wider tracks (more confident)
		score := -distance + float64(width)/10.0

		if score > bestScore {
			bestScore = score
			best = &filtered[i]
		}
	}

	return best
}

func abs(x int) int {
	if x < 0 {
		return -x
	}
	return x
}

func checkFinishLine(img *gocv.Mat, y int) bool {
	if img.Empty() {
		return false
	}

	width := img.Cols()
	height := img.Rows()

	if y >= height {
		return false
	}

	row := img.Row(y)
	defer row.Close()

	// Look for two distinct black regions in the middle third
	middleStart := width / 3
	middleEnd := 2 * width / 3

	blackRegions := 0
	inBlackRegion := false
	regionStart := 0

	for x := middleStart; x < middleEnd; x++ {
		pixel := row.GetUCharAt(0, x)
		if pixel == 0 {
			if !inBlackRegion {
				blackRegions++
				regionStart = x
				inBlackRegion = true
			}
		} else {
			if inBlackRegion {
				regionWidth := x - regionStart
				// Each black line should be substantial (not just noise)
				if regionWidth < 5 {
					blackRegions-- // Too small, not a real line
				}
				inBlackRegion = false
			}
		}
	}

	// Need exactly 2 substantial black regions
	return blackRegions == 2
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

	// PID controller for steering (tuned for smooth driving)
	pid := NewPIDController(0.15, 0.01, 0.05)

	// Track variables
	preferredX := imgWidth / 2
	lastValidMiddleX := preferredX
	consecutiveLostFrames := 0
	maxLostFrames := 15

	// Track width tracking for better filtering
	avgTrackWidth := 300 // Start with reasonable default
	widthAlpha := 0.3    // Smoothing factor

	// For PID timing
	lastTime := time.Now()

	for {
		frameStart := time.Now()

		if ok := cam.Read(&buf); !ok {
			log.Warn().Msg("Error reading from camera")
			continue
		}
		if buf.Empty() {
			continue
		}

		currentWidth := buf.Cols()
		currentHeight := buf.Rows()

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
			// Use adaptive threshold for better lighting handling
			gocv.AdaptiveThreshold(buf, &buf, 255, gocv.AdaptiveThresholdGaussian, gocv.ThresholdBinary, 51, 8)

			// Clean up noise
			kernel := gocv.GetStructuringElement(gocv.MorphRect, image.Pt(3, 3))
			gocv.MorphologyEx(buf, &buf, gocv.MorphClose, kernel)
			gocv.Erode(buf, &buf, kernel)
			kernel.Close()
		}

		// Check for finish line
		finishLineDetected := false
		for yOffset := 0; yOffset < 40; yOffset += 10 {
			if checkFinishLine(&buf, currentHeight-20-yOffset) {
				finishLineDetected = true
				break
			}
		}

		// Multi-line scan for better track detection
		type TrackSample struct {
			Y      int
			Center int
			Width  int
		}

		samples := []TrackSample{}

		// Scan multiple horizontal lines, weighted by importance
		scanLines := []int{
			currentHeight - 20, // Very close (most important)
			currentHeight - 40,
			currentHeight - 60,
			currentHeight - 80,
			currentHeight - 100,
			currentHeight - 120,
		}

		for _, y := range scanLines {
			if y < 0 || y >= currentHeight {
				continue
			}

			horizontalSlice := buf.Region(image.Rect(0, y, currentWidth, y+1))
			sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)

			// Get best slice at this Y
			bestSlice := getBestTrackSlice(sliceDescriptors, preferredX, avgTrackWidth)

			if bestSlice != nil {
				centerX := (bestSlice.Start + bestSlice.End) / 2
				width := bestSlice.End - bestSlice.Start
				samples = append(samples, TrackSample{Y: y, Center: centerX, Width: width})
			}

			horizontalSlice.Close()
		}

		var targetX int
		var foundTrack bool

		if len(samples) > 0 {
			consecutiveLostFrames = 0
			foundTrack = true

			// Weighted average based on Y position (closer = more important)
			totalWeight := 0.0
			weightedSum := 0.0

			for _, sample := range samples {
				// Weight: closer to car is exponentially more important
				distanceFromCar := currentHeight - sample.Y
				weight := math.Exp(-float64(distanceFromCar) / 50.0) // Decay over 50 pixels

				weightedSum += float64(sample.Center) * weight
				totalWeight += weight

				// Update average track width
				avgTrackWidth = int(float64(avgTrackWidth)*(1-widthAlpha) + float64(sample.Width)*widthAlpha)
			}

			if totalWeight > 0 {
				targetX = int(weightedSum / totalWeight)
			} else {
				targetX = samples[0].Center
			}

			// Smooth the target
			preferredX = preferredX*70/100 + targetX*30/100
			lastValidMiddleX = preferredX

		} else {
			consecutiveLostFrames++
			foundTrack = false

			if consecutiveLostFrames < maxLostFrames {
				// Use last known position, gradually drift to center
				preferredX = lastValidMiddleX*80/100 + (currentWidth/2)*20/100
			} else {
				// Completely lost, reset to center
				preferredX = currentWidth / 2
			}
		}

		// Calculate steering error (normalized -1 to 1)
		imageCenter := currentWidth / 2
		maxError := currentWidth / 3 // Max error for full steering
		rawError := float64(preferredX-imageCenter) / float64(maxError)

		// Clamp error
		if rawError > 1.0 {
			rawError = 1.0
		} else if rawError < -1.0 {
			rawError = -1.0
		}

		// Dead zone - ignore small errors to prevent wobble
		deadZone := 0.05
		if math.Abs(rawError) < deadZone {
			rawError = 0
		}

		// Apply PID for smooth steering
		dt := time.Since(lastTime).Seconds()
		if dt > 0.1 {
			dt = 0.1 // Cap dt
		}
		steeringOutput := pid.Update(rawError, dt)

		// Clamp steering output
		if steeringOutput > 1.0 {
			steeringOutput = 1.0
		} else if steeringOutput < -1.0 {
			steeringOutput = -1.0
		}

		lastTime = time.Now()

		// Log steering info
		if foundTrack {
			log.Debug().
				Float64("error", rawError).
				Float64("steering", steeringOutput).
				Int("targetX", preferredX).
				Int("samples", len(samples)).
				Int("trackWidth", avgTrackWidth).
				Msg("Steering")
		} else {
			log.Debug().Int("lostFrames", consecutiveLostFrames).Msg("Track lost")
		}

		// Prepare canvas for debugging
		canvasObjects := make([]*pb_output.CanvasObject, 0)

		// Draw track samples
		for _, sample := range samples {
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(sample.Center),
							Y: uint32(sample.Y),
						},
						Radius: 3,
					},
				},
			})
		}

		// Draw target line
		if foundTrack {
			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Line_{
					Line: &pb_output.CanvasObject_Line{
						Start: &pb_output.CanvasObject_Point{
							X: uint32(preferredX),
							Y: uint32(currentHeight - 20),
						},
						End: &pb_output.CanvasObject_Point{
							X: uint32(preferredX),
							Y: uint32(currentHeight),
						},
					},
				},
			})
		}

		// Draw center line
		canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
			Object: &pb_output.CanvasObject_Line_{
				Line: &pb_output.CanvasObject_Line{
					Start: &pb_output.CanvasObject_Point{
						X: uint32(imageCenter),
						Y: 0,
					},
					End: &pb_output.CanvasObject_Point{
						X: uint32(imageCenter),
						Y: uint32(currentHeight),
					},
				},
			},
		})

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

		// Add steering indicator
		steeringX := imageCenter + int(steeringOutput*float64(currentWidth)/4)
		canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
			Object: &pb_output.CanvasObject_Circle_{
				Circle: &pb_output.CanvasObject_Circle{
					Center: &pb_output.CanvasObject_Point{
						X: uint32(steeringX),
						Y: uint32(currentHeight - 10),
					},
					Radius: 5,
				},
			},
		})

		canvas := pb_output.Canvas{
			Objects: canvasObjects,
			Width:   uint32(currentWidth),
			Height:  uint32(currentHeight),
		}

		compressionParams := []int{gocv.IMWriteJpegQuality, 25}
		imgBytes, err := gocv.IMEncodeWithParams(".jpg", buf, compressionParams)
		if err != nil {
			log.Err(err).Msg("Error encoding image")
			continue
		}

		horizontalScans := make([]*pb_output.HorizontalScan, 0)

		if finishLineDetected {
			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  0,
				XRight: uint32(currentWidth),
				Y:      9999,
			})
			log.Info().Msg("FINISH LINE DETECTED")
		} else if foundTrack && len(samples) > 0 {
			// Send the closest scan for driving
			closestSample := samples[len(samples)-1]
			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  uint32(closestSample.Center - closestSample.Width/2),
				XRight: uint32(closestSample.Center + closestSample.Width/2),
				Y:      uint32(closestSample.Y),
			})
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

		// Maintain stable framerate
		frameTime := time.Since(frameStart)
		if frameTime < 33*time.Millisecond { // ~30 FPS
			time.Sleep(33*time.Millisecond - frameTime)
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
