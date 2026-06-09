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

type ScanCandidate struct {
	Start int
	End   int
	Y     int
	Score float64
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

func clampInt(v int, min int, max int) int {
	if v < min {
		return min
	}
	if v > max {
		return max
	}
	return v
}

func widthOf(desc SliceDescriptor) int {
	return desc.End - desc.Start
}

func centerOf(desc SliceDescriptor) int {
	return (desc.Start + desc.End) / 2
}

func isReasonableTrackWidth(width int, previousWidth float64, imgWidth int) bool {
	// Absolute safety limits.
	// Too small = glare/noise/tiny white patch.
	// Too large = glare/whole image washed out.
	absoluteMin := 120
	absoluteMax := int(float64(imgWidth) * 0.92)

	if width < absoluteMin || width > absoluteMax {
		return false
	}

	// Adaptive limits from previous good track width.
	// Generous so corners are not skipped.
	if previousWidth > 0 {
		minAdaptive := previousWidth * 0.45
		maxAdaptive := previousWidth * 1.45

		if float64(width) < minAdaptive || float64(width) > maxAdaptive {
			return false
		}
	}

	return true
}

func scoreCandidate(desc SliceDescriptor, y int, preferredX int, previousWidth float64, imgHeight int) float64 {
	width := float64(widthOf(desc))
	center := float64(centerOf(desc))

	score := 0.0

	// Prefer slices containing the previous/expected center.
	if preferredX > desc.Start && preferredX < desc.End {
		score += 1000.0
	}

	// Prefer center close to previous center.
	centerDistance := math.Abs(center - float64(preferredX))
	score -= centerDistance * 1.5

	// Prefer widths close to previous good width, but not too aggressively.
	if previousWidth > 0 {
		widthDistance := math.Abs(width - previousWidth)
		score -= widthDistance * 0.8
	}

	// Prefer lower scan lines slightly, because they are closer to the car.
	yPreference := float64(y) / float64(imgHeight)
	score += yPreference * 100.0

	// Prefer useful track widths around normal observed values.
	// Your normal logs were roughly 420-540.
	if width >= 350 && width <= 560 {
		score += 150.0
	}

	return score
}

func getBestCandidateAtY(buf *gocv.Mat, y int, preferredX int, previousWidth float64) *ScanCandidate {
	imgWidth := buf.Cols()
	imgHeight := buf.Rows()

	if y < 0 || y >= imgHeight-1 {
		return nil
	}

	horizontalSlice := buf.Region(image.Rect(0, y, imgWidth, y+1))
	defer horizontalSlice.Close()

	sliceDescriptors := getConsecutiveWhitePointsFromSlice(&horizontalSlice)

	var best *ScanCandidate = nil

	for _, desc := range sliceDescriptors {
		width := widthOf(desc)

		if !isReasonableTrackWidth(width, previousWidth, imgWidth) {
			continue
		}

		score := scoreCandidate(desc, y, preferredX, previousWidth, imgHeight)

		candidate := ScanCandidate{
			Start: desc.Start,
			End:   desc.End,
			Y:     y,
			Score: score,
		}

		if best == nil || candidate.Score > best.Score {
			best = &candidate
		}
	}

	return best
}

func findTrackScans(buf *gocv.Mat, preferredX int, previousWidth float64, baseSliceY int) []ScanCandidate {
	imgHeight := buf.Rows()

	// Multiple scan lines.
	// Lower lines are for normal driving.
	// Higher lines help prepare for corners.
	scanYs := []int{
		int(float64(imgHeight) * 0.72),
		int(float64(imgHeight) * 0.66),
		int(float64(imgHeight) * 0.60),
		int(float64(imgHeight) * 0.54),
		int(float64(imgHeight) * 0.48),
		int(float64(imgHeight) * 0.42),
	}

	// Also keep the original base sliceY behavior.
	scanYs = append(scanYs, baseSliceY)

	candidates := make([]ScanCandidate, 0)

	for _, y := range scanYs {
		y = clampInt(y, 0, imgHeight-2)

		candidate := getBestCandidateAtY(buf, y, preferredX, previousWidth)
		if candidate != nil {
			candidates = append(candidates, *candidate)
		}
	}

	if len(candidates) == 0 {
		return candidates
	}

	// Sort manually by score descending.
	for i := 0; i < len(candidates); i++ {
		for j := i + 1; j < len(candidates); j++ {
			if candidates[j].Score > candidates[i].Score {
				candidates[i], candidates[j] = candidates[j], candidates[i]
			}
		}
	}

	// Return best 3 scans.
	// First one is the one the controller should mainly use.
	if len(candidates) > 3 {
		candidates = candidates[:3]
	}

	return candidates
}

func rowBlackRatioInTrack(buf *gocv.Mat, y int, xLeft int, xRight int) float64 {
	if y < 0 || y >= buf.Rows() {
		return 0.0
	}

	xLeft = clampInt(xLeft, 0, buf.Cols()-1)
	xRight = clampInt(xRight, 0, buf.Cols()-1)

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

func detectFinishLineInTrack(buf *gocv.Mat, scan ScanCandidate) bool {
	imgHeight := buf.Rows()

	// Look inside the detected white track area.
	// Slightly shrink the edges so black borders do not count.
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

		// Black finish bars should cover a big part of the white track.
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

					// Count separate black bars.
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

		scans := findTrackScans(&buf, preferredX, previousWidth, sliceY)

		finishLineDetected := false

		if len(scans) > 0 {
			best := scans[0]
			middleX := (best.Start + best.End) / 2
			width := best.End - best.Start

			preferredX = middleX
			previousWidth = float64(width)

			finishLineDetected = detectFinishLineInTrack(&buf, best)

			if finishLineDetected {
				log.Info().Int("xLeft", best.Start).Int("xRight", best.End).Int("width", width).Msg("FINISH LINE DETECTED")
			}
		}

		canvasObjects := make([]*pb_output.CanvasObject, 0)

		for _, scan := range scans {
			middleX := (scan.Start + scan.End) / 2

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(scan.Start),
							Y: uint32(scan.Y),
						},
						Radius: 1,
					},
				},
			})

			canvasObjects = append(canvasObjects, &pb_output.CanvasObject{
				Object: &pb_output.CanvasObject_Circle_{
					Circle: &pb_output.CanvasObject_Circle{
						Center: &pb_output.CanvasObject_Point{
							X: uint32(scan.End),
							Y: uint32(scan.Y),
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
							Y: uint32(scan.Y),
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

		if len(scans) > 0 {
			best := scans[0]

			scanY := uint32(best.Y)

			// Keep XLeft/XRight normal.
			// Only mark finish line through Y.
			if finishLineDetected {
				scanY = 9999
			}

			// IMPORTANT:
			// Send only ONE scan to the controller.
			// Multiple scans make the car unstable.
			horizontalScans = append(horizontalScans, &pb_output.HorizontalScan{
				XLeft:  uint32(best.Start),
				XRight: uint32(best.End),
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
