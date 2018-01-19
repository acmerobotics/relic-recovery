package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.config.Config;

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

@Config
public class OldCryptoboxTracker extends Tracker {
    public static final String TAG = "OldCryptoboxTracker";

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 80;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 112, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 80;
    public static int BLUE_UPPER_HUE = 124, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    // brown HSV range
    public static int BROWN_LOWER_HUE = 0, BROWN_LOWER_SAT = 31, BROWN_LOWER_VALUE = 27;
    public static int BROWN_UPPER_HUE = 19, BROWN_UPPER_SAT = 94, BROWN_UPPER_VALUE = 104;

    // gray HSV range
    public static int GRAY_LOWER_HUE = 66, GRAY_LOWER_SAT = 3, GRAY_LOWER_VALUE = 121;
    public static int GRAY_UPPER_HUE = 126, GRAY_UPPER_SAT = 69, GRAY_UPPER_VALUE = 210;

    // binary morphology kernel sizes
    public static int OPEN_KERNEL_SIZE = 5;
    public static int CLOSE_KERNEL_SIZE = 5;

    public static double MAX_BLOB_ASPECT_RATIO = 0.5;
    public static int MIN_BLOB_SIZE = 250;
    public static double MAX_ASPECT_RATIO_ERROR = 0.2;
    public static double MIN_RECT_FILL = 0.85;

    public static final double ACTUAL_RAIL_GAP = 7.5; // in
    public static final double ACTUAL_GLYPH_SIZE = 6.0; // in

    private CryptoboxResult latestResult;
    private List<Glyph> latestGlyphs;
    private List<Rail> latestRawRails;
    private int actualWidth, actualHeight;
    private Mat resized, hsv, red, blue, brown, gray;
    private Mat morph, hierarchy, openKernel, closeKernel;
    private Mat redMorphClose, redMorphOpen, blueMorphClose, blueMorphOpen;
    private int openKernelSize, closeKernelSize;
    private boolean useExtendedTracking, initialized;
    private double focalLengthPx;
    private VisionCamera.Properties properties;

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

    public OldCryptoboxTracker(boolean useExtendedTracking) {
        this.useExtendedTracking = useExtendedTracking;
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
    public List<Rail> findRailsFromMask(CryptoboxColor color, Mat mask) {
        List<Rail> rails = new ArrayList<>();

        Mat morphOpen, morphClose;
        String prefix;
        if (color == CryptoboxColor.RED) {
            prefix = "red";
            morphOpen = redMorphOpen;
            morphClose = redMorphClose;
        } else {
            prefix = "blue";
            morphOpen = blueMorphOpen;
            morphClose = blueMorphClose;
        }

        Imgproc.morphologyEx(mask, morphOpen, Imgproc.MORPH_OPEN, openKernel);

        addIntermediate(prefix + "MorphOpen", morphOpen);

        Imgproc.morphologyEx(morphOpen, morphClose, Imgproc.MORPH_CLOSE, closeKernel);

        addIntermediate(prefix + "MorphClose", morphClose);

        List<MatOfPoint> railContours = new ArrayList<>();
        Imgproc.findContours(morphClose, railContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

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
            rails = VisionUtil.nonMaximumSuppression(rails, 3 * meanRailGap / 8);
        }

        return rails;
    }

    @Override
    public void init(VisionCamera camera) {
        this.properties = camera.getProperties();
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        actualWidth = frame.cols() / 4;
        actualHeight = frame.rows() / 4;

        focalLengthPx = properties.getHorizontalFocalLengthPx(actualWidth);

        if (!initialized) {
            resized = new Mat();
            hsv = new Mat();
            red = new Mat();
            blue = new Mat();
            brown = new Mat();
            gray = new Mat();
            morph = new Mat();
            hierarchy = new Mat();

            redMorphClose = new Mat();
            redMorphOpen = new Mat();
            blueMorphClose = new Mat();
            blueMorphOpen = new Mat();

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

        addIntermediate("blurred", resized);

        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar redLowerHsv = new Scalar(RED_LOWER_HUE, RED_LOWER_SAT, RED_LOWER_VALUE);
        Scalar redUpperHsv = new Scalar(RED_UPPER_HUE, RED_UPPER_SAT, RED_UPPER_VALUE);
        VisionUtil.smartHsvRange(hsv, redLowerHsv, redUpperHsv, red);

        Scalar blueLowerHsv = new Scalar(BLUE_LOWER_HUE, BLUE_LOWER_SAT, BLUE_LOWER_VALUE);
        Scalar blueUpperHsv = new Scalar(BLUE_UPPER_HUE, BLUE_UPPER_SAT, BLUE_UPPER_VALUE);
        VisionUtil.smartHsvRange(hsv, blueLowerHsv, blueUpperHsv, blue);

        List<Double> rails = new ArrayList<>();
        List<Rail> rawRails = new ArrayList<>();

        CryptoboxColor cryptoboxColor = CryptoboxColor.UNKNOWN;

        if (latestRawRails == null) {
            latestRawRails = new ArrayList<>();
        } else {
            latestRawRails.clear();
        }

        addIntermediate("red", red);

        List<Rail> rawRedRails = findRailsFromMask(CryptoboxColor.RED, red);

        addIntermediate("blue", blue);

        List<Rail> rawBlueRails = findRailsFromMask(CryptoboxColor.BLUE, blue);

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
            rawRails.addAll(rawRedRails);
        } else if (rawBlueRails.size() > rawRedRails.size()) {
            cryptoboxColor = CryptoboxColor.BLUE;
            rails.addAll(blueRails);
            rawRails.addAll(rawBlueRails);
        } else {
            int redCount = Core.countNonZero(red);
            int blueCount = Core.countNonZero(blue);

            if (redCount > blueCount) {
                cryptoboxColor = CryptoboxColor.RED;
                rails.addAll(redRails);
                rawRails.addAll(rawRedRails);
            } else {
                cryptoboxColor = CryptoboxColor.BLUE;
                rails.addAll(blueRails);
                rawRails.addAll(rawBlueRails);
            }
        }

        if (rawRails.size() > 0) {
            double meanRailWidth = 0;
            for (Rail rail : rawRails) {
                meanRailWidth += Imgproc.boundingRect(rail.contour).width;
            }
            meanRailWidth /= rawRails.size();
            rails = VisionUtil.nonMaximumSuppression(rails, 2.5 * meanRailWidth);
        }

        List<Double> glyphRails = new ArrayList<>();
        if (useExtendedTracking) {
            Scalar brownLowerHsv = new Scalar(BROWN_LOWER_HUE, BROWN_LOWER_SAT, BROWN_LOWER_VALUE);
            Scalar brownUpperHsv = new Scalar(BROWN_UPPER_HUE, BROWN_UPPER_SAT, BROWN_UPPER_VALUE);
            VisionUtil.smartHsvRange(hsv, brownLowerHsv, brownUpperHsv, brown);

            Scalar grayLowerHsv = new Scalar(GRAY_LOWER_HUE, GRAY_LOWER_SAT, GRAY_LOWER_VALUE);
            Scalar grayUpperHsv = new Scalar(GRAY_UPPER_HUE, GRAY_UPPER_SAT, GRAY_UPPER_VALUE);
            VisionUtil.smartHsvRange(hsv, grayLowerHsv, grayUpperHsv, gray);

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

            if (brownRails.size() > 1) {
                glyphRails = VisionUtil.nonMaximumSuppression(glyphRails, 3 * getMeanRailGap(brownRails) / 8);
            } else if (grayRails.size() > 1) {
                glyphRails = VisionUtil.nonMaximumSuppression(glyphRails, 3 * getMeanRailGap(grayRails) / 8);
            }
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
            rails = VisionUtil.nonMaximumSuppression(rails, 3 * meanRailGap / 8);
        }

        Collections.sort(rails);

        if (rails.size() == 2 || rails.size() == 3) {
            double meanRailGap = getMeanRailGap(rails);
            if (rails.get(0) < meanRailGap) {
                while (actualWidth - rails.get(rails.size() - 1) > meanRailGap && rails.size() < 4) {
                    // add extra rail on the left
                    rails.add(0, rails.get(0) - meanRailGap);
                }
            } else if (actualWidth - rails.get(rails.size() - 1) < meanRailGap) {
                while (rails.get(0) > meanRailGap && rails.size() < 4) {
                    // add extra rail on the right
                    rails.add(rails.get(rails.size() - 1) + meanRailGap);
                }
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
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
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
