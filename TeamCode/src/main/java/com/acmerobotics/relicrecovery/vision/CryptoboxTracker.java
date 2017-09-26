package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_LOWER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_LOWER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_LOWER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_UPPER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_UPPER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.BLUE_UPPER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.CLOSE_KERNEL_SIZE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.OPEN_KERNEL_SIZE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_LOWER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_LOWER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_LOWER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_UPPER_HUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_UPPER_SAT;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.RED_UPPER_VALUE;
import static com.acmerobotics.relicrecovery.vision.VisionConstants.SMALL_DIMENSION;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class CryptoboxTracker implements Tracker {
    private CryptoboxResult lastBlueResult, lastRedResult;
    private int actualWidth, actualHeight;
    private Paint paint;
    private Mat resized, hsv, red, blue, temp, morphedMask, morphedMask2, openKernel, closeKernel, hierarchy;

    public static class CryptoboxResult {
        public final List<MatOfPoint> contours;
        public final List<Double> lineXs;
        public final double timestamp;

        public CryptoboxResult(List<MatOfPoint> contours, List<Double> lineXs, double timestamp) {
            this.contours = contours;
            this.lineXs = lineXs;
            this.timestamp = timestamp;
        }
    }

    public CryptoboxTracker() {
        paint = new Paint();
        paint.setStrokeWidth(5.0f);
        paint.setStyle(Paint.Style.STROKE);
    }

    private void smartHsvRange(Mat src, Scalar lowerHsv, Scalar upperHsv, Mat dest) {
        if (lowerHsv.val[0] > upperHsv.val[0]) {
            Core.inRange(src, lowerHsv, new Scalar(180, upperHsv.val[1], upperHsv.val[2]), dest);
            if (temp == null) {
                temp = new Mat();
            }
            Core.inRange(src, new Scalar(0, lowerHsv.val[0], lowerHsv.val[1]), upperHsv, temp);
            Core.bitwise_or(dest, temp, dest);
        } else {
            Core.inRange(src, lowerHsv, upperHsv, dest);
        }
    }


    private CryptoboxResult findCryptobox(Mat src, double timestamp) {
        List<MatOfPoint> contours = new ArrayList<>();

        if (morphedMask == null) {
            morphedMask = new Mat();
            morphedMask2 = new Mat();
            openKernel = Mat.ones(OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE, CvType.CV_8U);
            closeKernel = Mat.ones(CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE, CvType.CV_8U);
            hierarchy = new Mat();
        }

        Imgproc.morphologyEx(src, morphedMask, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(morphedMask, morphedMask2, Imgproc.MORPH_CLOSE, closeKernel);
        Imgproc.findContours(morphedMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Double> lineXs = new ArrayList<>();
        int i = 0;
        while (i < contours.size()) {
            MatOfPoint contour = contours.get(i);
            int area = (int) Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            if (rect.width / rect.height > 0.5 || area < 750) {
                contours.remove(i);
                continue;
            }
            Mat line = new Mat();
            Imgproc.fitLine(contour, line, Imgproc.CV_DIST_L2, 0, 0.1, 0.1);
            float[] params = new float[4];
            line.get(0, 0, params);
            lineXs.add((double) params[2]);
            i++;
        }

        if (lineXs.size() >= 4) {
            Collections.sort(lineXs);
            double width = lineXs.get(lineXs.size() - 1) - lineXs.get(0);
            double meanColumnWidth = width / 4;
            // merge lines that are close together
            List<Double> mergedXs = new ArrayList<>();
            double total = lineXs.get(0);
            int count = 1;
            for (int j = 1; j < lineXs.size(); j++) {
                if (lineXs.get(j) - lineXs.get(j - 1) > meanColumnWidth / 4) {
                    mergedXs.add(total / count);
                    total = 0;
                    count = 0;
                }
                total += lineXs.get(j);
                count++;
            }
            mergedXs.add(total / count);
            return new CryptoboxResult(contours, mergedXs, timestamp);
        }

        return new CryptoboxResult(contours, lineXs, timestamp);
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        if (resized == null) {
            resized = new Mat();
            hsv = new Mat();
            red = new Mat();
            blue = new Mat();
        }

        int width = frame.width(), height = frame.height();
        int newWidth, newHeight;
        if (height < width) {
            newHeight = SMALL_DIMENSION;
            newWidth = (int) ((double) width / height * newHeight);
        } else {
            newWidth = SMALL_DIMENSION;
            newHeight = (int) ((double) height / width * newWidth);
        }

        actualWidth = newWidth;
        actualHeight = newHeight;

        Imgproc.resize(frame, resized, new Size(newWidth, newHeight));

        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar redLowerHsv = new Scalar(RED_LOWER_HUE, RED_LOWER_SAT, RED_LOWER_VALUE);
        Scalar redUpperHsv = new Scalar(RED_UPPER_HUE, RED_UPPER_SAT, RED_UPPER_VALUE);
        smartHsvRange(hsv, redLowerHsv, redUpperHsv, red);

        Scalar blueLowerHsv = new Scalar(BLUE_LOWER_HUE, BLUE_LOWER_SAT, BLUE_LOWER_VALUE);
        Scalar blueUpperHsv = new Scalar(BLUE_UPPER_HUE, BLUE_UPPER_SAT, BLUE_UPPER_VALUE);
        smartHsvRange(hsv, blueLowerHsv, blueUpperHsv, blue);

        lastRedResult = findCryptobox(red, timestamp);
        lastBlueResult = findCryptobox(blue, timestamp);
    }

    public void drawCryptobox(Canvas canvas, CryptoboxResult result, Paint paint) {
        for (MatOfPoint contour : result.contours) {
            VisionUtil.drawContour(canvas, contour, paint);
        }
        for (Double lineX : result.lineXs) {
            VisionUtil.drawLine(canvas, new Point(lineX, 0), new Point(lineX, actualHeight), paint);
        }
    }

    @Override
    public synchronized void drawOverlay(Canvas canvas, int imageWidth, int imageHeight) {
        canvas.translate(imageWidth / 2.0f, imageHeight / 2.0f);
        canvas.scale((float) imageWidth / actualWidth, (float) imageHeight / actualHeight);
        canvas.translate(-actualWidth / 2.0f, -actualHeight / 2.0f);

        if (lastRedResult != null) {
            paint.setColor(Color.RED);
            drawCryptobox(canvas, lastRedResult, paint);
        }

        if (lastBlueResult != null) {
            paint.setColor(Color.BLUE);
            drawCryptobox(canvas, lastBlueResult, paint);
        }
    }

    public synchronized CryptoboxResult getLastBlueResult() {
        return lastBlueResult;
    }

    public synchronized CryptoboxResult getLastRedResult() {
        return lastRedResult;
    }
}
