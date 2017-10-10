package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

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

    private CryptoboxResult latestResult;
    private int actualWidth, actualHeight;
    private Paint paint;
    private Mat resized, hsv, red, blue, brown, gray;
    private Mat temp, morph, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private boolean useExtendedTracking, initialized;
    private double fov; // rad

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

    public CryptoboxTracker(boolean useExtendedTracking) {
        paint = new Paint();
        paint.setStrokeWidth(5.0f);
        paint.setStyle(Paint.Style.STROKE);
        this.useExtendedTracking = useExtendedTracking;
    }

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

    public static List<Double> nonMaximalSuppression(List<Double> values, double threshold) {
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

    public List<Double> analyzeCryptobox(Mat mask) {
        List<Double> rails = new ArrayList<>();

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
            rails.add(centroidX);
        }

        return rails;
    }

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

        if (leftOverflow) {
            rails.add(0, rails.get(0) - meanRailGap);
        }

        if (rightOverflow) {
            rails.add(rails.get(rails.size() - 1) + meanRailGap);
        }

        if (rails.size() > 0) {
            rails = nonMaximalSuppression(rails, 3 * meanRailGap / 8);
        }

        return rails;
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        if (!initialized) {
            CameraCalibration cameraCalibration = CameraDevice.getInstance().getCameraCalibration();
            fov = cameraCalibration.getFieldOfViewRads().getData()[0];
            initialized = true;
        }

        if (resized == null) {
            resized = new Mat();
            hsv = new Mat();
            red = new Mat();
            blue = new Mat();
            brown = new Mat();
            gray = new Mat();
            morph = new Mat();
            hierarchy = new Mat();
        }

        actualWidth = frame.cols() / 4;
        actualHeight = frame.rows() / 4;

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

        List<Double> redRails = analyzeCryptobox(red);
        List<Double> blueRails = analyzeCryptobox(blue);

        // TODO: do some weird checking

        rails.addAll(redRails);
        rails.addAll(blueRails);

        if (useExtendedTracking) {
            Scalar brownLowerHsv = new Scalar(BROWN_LOWER_HUE, BROWN_LOWER_SAT, BROWN_LOWER_VALUE);
            Scalar brownUpperHsv = new Scalar(BROWN_UPPER_HUE, BROWN_UPPER_SAT, BROWN_UPPER_VALUE);
            smartHsvRange(hsv, brownLowerHsv, brownUpperHsv, brown);

            Scalar grayLowerHsv = new Scalar(GRAY_LOWER_HUE, GRAY_LOWER_SAT, GRAY_LOWER_VALUE);
            Scalar grayUpperHsv = new Scalar(GRAY_UPPER_HUE, GRAY_UPPER_SAT, GRAY_UPPER_VALUE);
            smartHsvRange(hsv, grayLowerHsv, grayUpperHsv, gray);

            List<Glyph> brownGlyphs = findGlyphs(brown);
            List<Glyph> grayGlyphs = findGlyphs(gray);

            List<Double> brownRails = findRailsFromGlyphs(brownGlyphs);
            List<Double> grayRails = findRailsFromGlyphs(grayGlyphs);

            rails.addAll(brownRails);
            rails.addAll(grayRails);
        }

        Collections.sort(rails);

        // TODO: final rail NMS

        // TODO: work on NMS first !!!
//        double distance = Double.NaN, offsetX = Double.NaN;
//        if (rails.size() > 1) {
//            double maxRail = rails.get(rails.size() - 1);
//            double minRail = rails.get(0);
//            double railGap = (maxRail - minRail) / (rails.size() - 1);
//            double focalLengthPx = (actualWidth * 0.5) / Math.tan(0.5 * fov);
//            distance = (ACTUAL_RAIL_GAP * focalLengthPx) / railGap;
//            if (rails.size() == 4) {
//                double center = (maxRail - minRail) / 2.0;
//                offsetX = ((0.5 * actualWidth - center) * distance) / focalLengthPx;
//            }
//        }

        latestResult = new CryptoboxResult(CryptoboxColor.UNKNOWN, rails, 0, 0, timestamp);
    }

    @Override
    public synchronized void drawOverlay(Canvas canvas, int imageWidth, int imageHeight) {
        if (latestResult != null) {
            paint.setStyle(Paint.Style.FILL_AND_STROKE);
            paint.setStrokeWidth(5.0f);
            paint.setTextSize(45.0f);
            paint.setColor(Color.YELLOW);

            canvas.drawText(String.format("%.2f, %.2f", latestResult.distance, latestResult.offsetX), 5, 100, paint);

            canvas.translate(imageWidth / 2.0f, imageHeight / 2.0f);
            canvas.scale((float) imageWidth / actualWidth, (float) imageHeight / actualHeight);
            canvas.translate(-actualWidth / 2.0f, -actualHeight / 2.0f);

            paint.setStyle(Paint.Style.STROKE);
            switch (latestResult.color) {
                case RED:
                    paint.setColor(Color.YELLOW);
                    break;
                case BLUE:
                    paint.setColor(Color.CYAN);
                    break;
                case UNKNOWN:
                    paint.setColor(Color.GRAY);
                    break;
            }

            for (double rail : latestResult.rails) {
                VisionUtil.drawLine(canvas, new Point(rail, 0), new Point(rail, imageHeight), paint);
            }

//            paint.setColor(Color.GREEN);
//            for (Rect rect : latestFullGlyphs) {
//                VisionUtil.drawRect(canvas, rect, paint);
//            }
//
//            paint.setColor(Color.MAGENTA);
//            for (Rect rect : latestPartialGlyphs) {
//                VisionUtil.drawRect(canvas, rect, paint);
//            }
        }
    }

    public synchronized CryptoboxResult getLatestResult() {
        return latestResult;
    }
}
