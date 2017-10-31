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

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 10/30/17.
 */

@Config
public class DynamicJewelTracker implements Tracker {
    public static int OPEN_KERNEL_SIZE = 5;
    public static int CLOSE_KERNEL_SIZE = 15;

    // red HSV range
    public static int RED_LOWER_HUE = 173, RED_LOWER_SAT = 100, RED_LOWER_VALUE = 50;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 81, BLUE_LOWER_SAT = 100, BLUE_LOWER_VALUE = 50;
    public static int BLUE_UPPER_HUE = 110, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    public static int MIN_BLOB_SIZE = 250;
    public static double MAX_ASPECT_RATIO_ERROR = 0.3;

    private Mat resized, hsv, red, blue;
    private Mat temp, morph, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private List<Rect> lastRedJewels, lastBlueJewels;
    private List<MatOfPoint> lastContours;
    private volatile boolean isLeftRed;

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

    private List<Rect> findJewels(Mat mask) {
        if (morph == null) {
            morph = new Mat();
            hierarchy = new Mat();
        }

        Imgproc.morphologyEx(mask, morph, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_CLOSE, closeKernel);

        List<MatOfPoint> jewelContours = new ArrayList<>();
        Imgproc.findContours(morph, jewelContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        lastContours.addAll(jewelContours);

        List<Rect> jewelRects = new ArrayList<>();
        for (MatOfPoint contour : jewelContours) {
            if (Imgproc.contourArea(contour) < MIN_BLOB_SIZE) {
                continue;
            }
            Rect rect = Imgproc.boundingRect(contour);
            if (Math.abs(rect.width / rect.height - 1) > MAX_ASPECT_RATIO_ERROR) {
                continue;
            }
            jewelRects.add(rect);
        }

        return jewelRects;
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        if (resized == null) {
            resized = new Mat();
            hsv = new Mat();
            red = new Mat();
            blue = new Mat();
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

        if (lastContours == null) {
            lastContours = new ArrayList<>();
        } else {
            lastContours.clear();
        }

        lastRedJewels = findJewels(red);
        lastBlueJewels = findJewels(blue);

        if (lastRedJewels.size() == 0 || lastBlueJewels.size() == 0) {
            return;
        }

        // TODO: consider doing a better check
        isLeftRed = lastRedJewels.get(0).x < lastBlueJewels.get(0).x;
    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight) {
        overlay.setScalingFactor(4);

        if (lastRedJewels != null) {
            for (MatOfPoint contour : lastContours) {
                overlay.strokeContour(contour, new Scalar(255, 255, 255), 10);
            }

            for (Rect redJewel : lastRedJewels) {
                overlay.strokeRect(redJewel, new Scalar(0, 0, 255), 10);
            }

            for (Rect blueJewel : lastBlueJewels) {
                overlay.strokeRect(blueJewel, new Scalar(255, 0, 0), 10);
            }
        }

        overlay.setScalingFactor(1);

        overlay.putText(
                isLeftRed ? "R / B" : "B / R",
                Overlay.TextAlign.LEFT,
                new Point(5, 50),
                new Scalar(0, 0, 255),
                45
        );
    }

    public synchronized boolean isLeftRed() {
        return isLeftRed;
    }

    public synchronized boolean isLeftBlue() {
        return !isLeftRed;
    }
}
