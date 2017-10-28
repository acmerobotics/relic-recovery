package com.acmerobotics.relicrecovery.vision;

import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;
import java.util.Locale;

import static com.acmerobotics.relicrecovery.vision.VisionConstants.ACTUAL_GLYPH_SIZE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.ACTUAL_RAIL_GAP;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_LOWER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_LOWER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_LOWER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_UPPER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_UPPER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_UPPER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BROWN_LOWER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BROWN_LOWER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BROWN_LOWER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BROWN_UPPER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BROWN_UPPER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BROWN_UPPER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.CLOSE_KERNEL_SIZE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.GRAY_LOWER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.GRAY_LOWER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.GRAY_LOWER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.GRAY_UPPER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.GRAY_UPPER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.GRAY_UPPER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.MAX_ASPECT_RATIO_ERROR;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.MAX_BLOB_ASPECT_RATIO;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.MIN_BLOB_SIZE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.MIN_RECT_FILL;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.OPEN_KERNEL_SIZE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_LOWER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_LOWER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_LOWER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_UPPER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_UPPER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_UPPER_VALUE;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class CryptoboxTracker implements Tracker {
    public static final String TAG = "CryptoboxTracker";

    // TODO: hack!
    public static boolean isUnitTest;

    private CryptoboxResult latestResult;
    private List<Glyph> latestGlyphs;
    private List<Rail> latestRawRails;
    private int actualWidth, actualHeight;
    private Mat resized, hsv, red, blue, brown, gray;
    private Mat temp, morph, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private boolean useExtendedTracking, initialized;
    private double focalLengthPx;

    public enum CryptoboxColor {
        BLUE,
        RED,
        UNKNOWN
    }

    public static class CryptoboxResult {
        public final CryptoboxColor color;
        public final List<Double> rails;
        public final double distance, offsetX;
        public final double timestamp;

        public CryptoboxResult(CryptoboxColor color, List<Double> rails, double distance, double offsetX, double timestamp) {
            this.color = color;
            this.rails = rails;
            this.distance = distance;
            this.offsetX = offsetX;
            this.timestamp = timestamp;
        }
    }

    public enum GlyphType {
        FULL,
        PARTIAL_WIDTH,
        PARTIAL_HEIGHT
    }

    public static class Glyph {
        public final GlyphType type;
        public final Rect rect;

        public Glyph(GlyphType type, Rect rect) {
            this.type = type;
            this.rect = rect;
        }
    }

    public static class Rail {
        public final double x;
        public final MatOfPoint contour;

        public Rail(double x, MatOfPoint contour) {
            this.x = x;
            this.contour = contour;
        }
    }

    public CryptoboxTracker(boolean useExtendedTracking) {
        this.useExtendedTracking = useExtendedTracking;
    }

    /**
     * Thresholds an HSV image. If the lower hue is greater than the upper hue, it thresholds from
     * 0 to the lower hue and from the upper hue to 180.
     * @param src
     * @param lowerHsv
     * @param upperHsv
     * @param dest
     */
    private void smartHsvRange(Mat src, Scalar lowerHsv, Scalar upperHsv, Mat dest) {
        if (lowerHsv.val[0] > upperHsv.val[0]) {
            Core.inRange(src, lowerHsv, new Scalar(180, upperHsv.val[1], upperHsv.val[2]), dest);
            if (temp == null) {
                temp = new Mat();
            }
            Core.inRange(src, new Scalar(0, lowerHsv.val[1], lowerHsv.val[2]), upperHsv, temp);
            Core.bitwise_or(dest, temp, dest);
        } else {
            Core.inRange(src, lowerHsv, upperHsv, dest);
        }
    }

    /**
     * Replace groups of values within the threshold with their means (thin out duplicate
     * detections). Note: this requires a sorted array and preserves the sorting.
     * @link https://www.pyimagesearch.com/2014/11/17/non-maximum-suppression-object-detection-python/
     * @param values
     * @param threshold
     */
    public static List<Double> nonMaximumSuppression(List<Double> values, double threshold) {
        Collections.sort(values);
        List<Double> outputValues = new ArrayList<>();
        double count = 1, total = values.get(0), lastValue = values.remove(0);
        for (double value : values) {
            if (value - lastValue > threshold) {
                outputValues.add(total / count);
                total = value;
                lastValue = value;
                count = 1;
            } else {
                total += value;
                count += 1;
            }
        }
        outputValues.add(total / count);
        return outputValues;
    }

    /**
     * Compute the mean rail gap given a sequence of rail x-coordinates
     * @param rails
     * @return mean rail gap
     */
    public static double getMeanRailGap(List<Double> rails) {
        if (rails.size() < 2) {
            throw new IllegalArgumentException("Two rails are required to compute the mean rail gap");
        }

        double min = rails.get(0), max = rails.get(0);
        for (int i = 1; i < rails.size(); i++) {
            double rail = rails.get(i);
            if (rail < min) {
                min = rail;
            }
            if (rail > max) {
                max = rail;
            }
        }

        return (max - min) / (rails.size() - 1);
    }

    /**
     * Detect cryptobox-rail-shaped color blobs in an image.
     * @param mask
     * @return detected rail x-coordinates
     */
    public List<Rail> analyzeCryptobox(Mat mask) {
        List<Rail> rails = new ArrayList<>();

        Imgproc.morphologyEx(mask, morph, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_CLOSE, closeKernel);

        List<MatOfPoint> railContours = new ArrayList<>();
        Imgproc.findContours(morph, railContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        for (MatOfPoint contour : railContours) {
            int area = (int) Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.width / rect.height > MAX_BLOB_ASPECT_RATIO || area < MIN_BLOB_SIZE) {
                continue;
            }
            Moments moments = Imgproc.moments(contour);
            double centroidX = moments.get_m10() / moments.get_m00();
            rails.add(new Rail(centroidX, contour));
        }

        return rails;
    }

    /**
     * Find glyphs in an image (including partial ones).
     * @param mask
     * @return detected glyphs
     */
    public List<Glyph> findGlyphs(Mat mask) {
        List<Glyph> glyphs = new ArrayList<>();

        Imgproc.morphologyEx(mask, morph, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_CLOSE, closeKernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(morph, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        int glyphWidthSum = 0, glyphWidthCount = 0;

        List<Rect> potentialGlyphs = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            double area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            double rectArea = rect.width * rect.height;
            double aspectRatio = (double) rect.height / rect.width;
            int aspectRatioRounded = (int) Math.round(aspectRatio);
            if (area / rectArea > MIN_RECT_FILL) {
                if (Math.abs(aspectRatioRounded - aspectRatio) < MAX_ASPECT_RATIO_ERROR) {
                    int y = rect.y, height = rect.height;
                    if (aspectRatioRounded > 1) {
                        height /= aspectRatioRounded;
                    }
                    glyphWidthSum += rect.width;
                    glyphWidthCount++;
                    for (int j = 0; j < aspectRatioRounded; j++) {
                        glyphs.add(new Glyph(GlyphType.FULL, new Rect(rect.x, y, rect.width, height)));
                        y += height;
                    }
                } else {
                    potentialGlyphs.add(rect);
                }
            }
        }

        if (glyphWidthCount > 0) {
            double meanGlyphSize = (double) glyphWidthSum / glyphWidthCount;

            for (Rect rect : potentialGlyphs) {
                int rightX = rect.x + rect.width;
                int bottomY = rect.y + rect.height;

                if (rect.x != 0 && rect.y != 0 && rightX != actualWidth && bottomY != actualHeight) {
                    continue;
                }

                double widthRatio = rect.width / meanGlyphSize;
                int widthRatioRounded = (int) Math.round(widthRatio);
                double heightRatio = rect.height / meanGlyphSize;
                int heightRatioRounded = (int) Math.round(heightRatio);

                if (Math.abs(widthRatio - widthRatioRounded) < MAX_ASPECT_RATIO_ERROR) {
                    glyphs.add(new Glyph(GlyphType.PARTIAL_WIDTH, rect));
                } else if (Math.abs(heightRatio - heightRatioRounded) < MAX_ASPECT_RATIO_ERROR) {
                    glyphs.add(new Glyph(GlyphType.PARTIAL_HEIGHT, rect));
                }
            }
        }

        return glyphs;
    }

    /**
     * Find rails based on glyph detections
     * @param glyphs
     * @return detected rail x-coordinates
     */
    public List<Double> findRailsFromGlyphs(List<Glyph> glyphs) {
        List<Double> rails = new ArrayList<>();

        double meanRailGap = 0, count = 0;
        boolean leftOverflow = false, rightOverflow = false;

        for (Glyph glyph : glyphs) {
            if (glyph.type == GlyphType.PARTIAL_WIDTH) {
                if (glyph.rect.x == 0) {
                    leftOverflow = true;
                } else {
                    rightOverflow = true;
                }
                continue;
            }
            Rect rect = glyph.rect;
            double glyphCenter = rect.x + rect.width / 2.0;
            double railGap = rect.width * ACTUAL_RAIL_GAP / ACTUAL_GLYPH_SIZE;
            double leftRail = glyphCenter - 0.5 * railGap;
            double rightRail = glyphCenter + 0.5 * railGap;
            rails.add(leftRail);
            rails.add(rightRail);

            meanRailGap += railGap;
            count++;
        }
        meanRailGap /= count;

        Collections.sort(rails);

        if (leftOverflow) {
            rails.add(0, rails.get(0) - meanRailGap);
        }

        if (rightOverflow) {
            rails.add(rails.get(rails.size() - 1) + meanRailGap);
        }

        if (rails.size() > 0) {
            rails = nonMaximumSuppression(rails, 3 * meanRailGap / 8);
        }

        return rails;
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        actualWidth = frame.cols() / 4;
        actualHeight = frame.rows() / 4;

        if (!initialized) {
            // TODO: this is a bad hack!! find a better way to do this
            if (isUnitTest) {
                focalLengthPx = 270.451191280832; // Moto G4 Play
            } else {
                CameraCalibration cameraCalibration = CameraDevice.getInstance().getCameraCalibration();
                double fov = cameraCalibration.getFieldOfViewRads().getData()[0];
                focalLengthPx = (actualWidth * 0.5) / Math.tan(0.5 * fov);
                System.out.println(focalLengthPx);
            }

            resized = new Mat();
            hsv = new Mat();
            red = new Mat();
            blue = new Mat();
            brown = new Mat();
            gray = new Mat();
            morph = new Mat();
            hierarchy = new Mat();

            initialized = true;
        }

        if (openKernel == null || openKernelSize != OPEN_KERNEL_SIZE) {
            if (openKernel != null) {
                openKernel.release();
            }
            openKernel = Mat.ones(OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE, CvType.CV_8U);
            openKernelSize = OPEN_KERNEL_SIZE;
        }

        if (closeKernel == null || closeKernelSize != CLOSE_KERNEL_SIZE) {
            if (closeKernel != null) {
                closeKernel.release();
            }
            closeKernel = Mat.ones(CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE, CvType.CV_8U);
            closeKernelSize = CLOSE_KERNEL_SIZE;
        }

        Imgproc.pyrDown(frame, resized);
        Imgproc.pyrDown(resized, resized);

        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar redLowerHsv = new Scalar(RED_LOWER_HUE, RED_LOWER_SAT, RED_LOWER_VALUE);
        Scalar redUpperHsv = new Scalar(RED_UPPER_HUE, RED_UPPER_SAT, RED_UPPER_VALUE);
        smartHsvRange(hsv, redLowerHsv, redUpperHsv, red);

        Scalar blueLowerHsv = new Scalar(BLUE_LOWER_HUE, BLUE_LOWER_SAT, BLUE_LOWER_VALUE);
        Scalar blueUpperHsv = new Scalar(BLUE_UPPER_HUE, BLUE_UPPER_SAT, BLUE_UPPER_VALUE);
        smartHsvRange(hsv, blueLowerHsv, blueUpperHsv, blue);

        List<Double> rails = new ArrayList<>();

        CryptoboxColor cryptoboxColor = CryptoboxColor.UNKNOWN;

        if (latestRawRails == null) {
            latestRawRails = new ArrayList<>();
        } else {
            latestRawRails.clear();
        }

        List<Rail> rawRedRails = analyzeCryptobox(red);
        List<Rail> rawBlueRails = analyzeCryptobox(blue);

        latestRawRails.addAll(rawRedRails);
        latestRawRails.addAll(rawBlueRails);

        List<Double> redRails = new ArrayList<>();
        for (Rail rawRail : rawRedRails) {
            redRails.add(rawRail.x);
        }

        List<Double> blueRails = new ArrayList<>();
        for (Rail rawRail : rawBlueRails) {
            blueRails.add(rawRail.x);
        }

        if (rawRedRails.size() > rawBlueRails.size()) {
            cryptoboxColor = CryptoboxColor.RED;
            rails.addAll(redRails);
        } else if (rawBlueRails.size() > rawRedRails.size()) {
            cryptoboxColor = CryptoboxColor.BLUE;
            rails.addAll(blueRails);
        } else {
            int redCount = Core.countNonZero(red);
            int blueCount = Core.countNonZero(blue);

            if (redCount > blueCount) {
                cryptoboxColor = CryptoboxColor.RED;
                rails.addAll(redRails);
            } else {
                cryptoboxColor = CryptoboxColor.BLUE;
                rails.addAll(blueRails);
            }
        }

        List<Double> glyphRails = new ArrayList<>();
        if (useExtendedTracking) {
            Scalar brownLowerHsv = new Scalar(BROWN_LOWER_HUE, BROWN_LOWER_SAT, BROWN_LOWER_VALUE);
            Scalar brownUpperHsv = new Scalar(BROWN_UPPER_HUE, BROWN_UPPER_SAT, BROWN_UPPER_VALUE);
            smartHsvRange(hsv, brownLowerHsv, brownUpperHsv, brown);

            Scalar grayLowerHsv = new Scalar(GRAY_LOWER_HUE, GRAY_LOWER_SAT, GRAY_LOWER_VALUE);
            Scalar grayUpperHsv = new Scalar(GRAY_UPPER_HUE, GRAY_UPPER_SAT, GRAY_UPPER_VALUE);
            smartHsvRange(hsv, grayLowerHsv, grayUpperHsv, gray);

            List<Glyph> brownGlyphs = findGlyphs(brown);
            List<Glyph> grayGlyphs = findGlyphs(gray);

            if (latestGlyphs == null) {
                latestGlyphs = new ArrayList<>();
            } else {
                latestGlyphs.clear();
            }
            latestGlyphs.addAll(brownGlyphs);
            latestGlyphs.addAll(grayGlyphs);

            List<Double> brownRails = findRailsFromGlyphs(brownGlyphs);
            List<Double> grayRails = findRailsFromGlyphs(grayGlyphs);

            glyphRails.addAll(brownRails);
            glyphRails.addAll(grayRails);
        }

        // combine all the rails
        if (glyphRails.size() == 0 || rails.size() == 0) {
            // no NMS needed
            rails.addAll(glyphRails);
        } else if (glyphRails.size() == 1 && rails.size() == 1) {
            // do nothing
        } else {
            // normal NMS
            double meanRailGap;
            if (rails.size() > 1) {
                meanRailGap = getMeanRailGap(rails);
            } else {
                meanRailGap = getMeanRailGap(glyphRails);
            }
            rails.addAll(glyphRails);
            rails = nonMaximumSuppression(rails, 3 * meanRailGap / 8);
        }

        Collections.sort(rails);

        if (rails.size() == 3) {
            double meanRailGap = getMeanRailGap(rails);
            if (rails.get(0) < meanRailGap && (actualWidth - rails.get(2)) > meanRailGap) {
                // likely extra rail on the left
                rails.add(0, rails.get(0) - meanRailGap);
            } else if (rails.get(0) > meanRailGap && (actualWidth - rails.get(2)) < meanRailGap) {
                // likely extra rail on the right
                rails.add(rails.get(2) + meanRailGap);
            }
        }

        // calculate the distance and horizontal offset of the cryptobox
        double distance = Double.NaN, offsetX = Double.NaN;
        if (rails.size() > 1) {
            double meanRailGap = getMeanRailGap(rails);
            distance = (ACTUAL_RAIL_GAP * focalLengthPx) / meanRailGap;
            if (rails.size() == 4) {
                double center = (Collections.max(rails) + Collections.min(rails)) / 2.0;
                offsetX = ((0.5 * actualWidth - center) * distance) / focalLengthPx;
            }
        }

        latestResult = new CryptoboxResult(cryptoboxColor, rails, distance, offsetX, timestamp);
    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight) {
        if (latestResult != null) {
            overlay.setScalingFactor(imageWidth / actualWidth);

            // draw rail contours
            if (latestRawRails != null) {
                for (Rail rawRail : latestRawRails) {
                    overlay.strokeContour(rawRail.contour, new Scalar(255, 255, 255), 5);
                }
            }

            // draw rails
            Scalar railColor = null;
            switch (latestResult.color) {
                case RED:
                    railColor = new Scalar(255, 0, 255);
                    break;
                case BLUE:
                    railColor = new Scalar(255, 255, 0);
                    break;
                case UNKNOWN:
                    railColor = new Scalar(0, 0, 0);
                    break;
            }

            for (double rail : latestResult.rails) {
                overlay.strokeLine(new Point(rail, 0), new Point(rail, actualHeight), railColor, 10);
            }

            // draw glyphs
            if (latestGlyphs != null) {
                for (Glyph glyph : latestGlyphs) {
                    switch (glyph.type) {
                        case FULL:
                            overlay.strokeRect(glyph.rect, new Scalar(0, 255, 0), 5);
                            break;
                        case PARTIAL_HEIGHT:
                        case PARTIAL_WIDTH:
                            // intentional fall through
                            overlay.strokeRect(glyph.rect, new Scalar(255, 0, 255), 5);
                            break;
                    }
                }
            }

            overlay.setScalingFactor(1);

            overlay.putText(
                    String.format(Locale.ENGLISH, "%.2f, %.2f", latestResult.distance, latestResult.offsetX),
                    Overlay.TextAlign.LEFT,
                    new Point(5, 50),
                    new Scalar(0, 0, 255),
                    45
            );
        }
    }

    public synchronized CryptoboxResult getLatestResult() {
        return latestResult;
    }
}
