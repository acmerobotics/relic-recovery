package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 10/30/17.
 */

@Config
public class DynamicJewelTracker extends Tracker {

    public static int OPEN_KERNEL_SIZE = 5;
    public static int CLOSE_KERNEL_SIZE = 11;

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 80;
    public static int RED_UPPER_HUE = 22, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 99, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 80;
    public static int BLUE_UPPER_HUE = 120, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    public static int MIN_BLOB_SIZE = 250;
    public static double MAX_ASPECT_RATIO_ERROR = 0.3;
    public static double MAX_ECCENTRICITY_ERROR = 0.3;

    public static double DIST_RATIO = 6.0 / 1.875; // distance between centers / radius
    public static double MAX_DIST_RATIO_ERROR = 0.3;

    private Mat resized, hsv, red, blue;
    private Mat temp, morph, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private List<RotatedRect> lastRedJewels, lastBlueJewels;
    private List<MatOfPoint> lastContours;
    private volatile JewelColor leftJewelColor = JewelColor.UNKNOWN;

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

    private List<RotatedRect> findJewelEllipses(Mat mask) {
        if (morph == null) {
            morph = new Mat();
            hierarchy = new Mat();
        }

        Imgproc.morphologyEx(mask, morph, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(morph, morph, Imgproc.MORPH_CLOSE, closeKernel);

        List<MatOfPoint> jewelContours = new ArrayList<>();
        Imgproc.findContours(morph, jewelContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        lastContours.addAll(jewelContours);

        List<RotatedRect> jewelEllipses = new ArrayList<>();
        for (MatOfPoint contour : jewelContours) {
            if (Imgproc.contourArea(contour) < MIN_BLOB_SIZE) {
                continue;
            }
            Rect rect = Imgproc.boundingRect(contour);
            if (Math.abs(rect.width / rect.height - 1) > MAX_ASPECT_RATIO_ERROR) {
                continue;
            }
            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            RotatedRect ellipse = Imgproc.fitEllipse(contour2f);
            if (Math.abs(ellipse.size.width / ellipse.size.height - 1) > MAX_ECCENTRICITY_ERROR) {
                continue;
            }
            jewelEllipses.add(ellipse);
        }

        return jewelEllipses;
    }

    @Override
    public void init(VisionCamera camera) {

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

        lastRedJewels = findJewelEllipses(red);
        lastBlueJewels = findJewelEllipses(blue);

        if (lastRedJewels.size() == 0 || lastBlueJewels.size() == 0) {
            leftJewelColor = JewelColor.UNKNOWN;
        } else {
            double minDistRatioError = Double.POSITIVE_INFINITY;
            RotatedRect bestRedJewel = null, bestBlueJewel = null;
            for (RotatedRect redJewel : lastRedJewels) {
                double redRadius = (redJewel.size.width + redJewel.size.height) / 4.0;
                for (RotatedRect blueJewel : lastBlueJewels) {
                    double blueRadius = (blueJewel.size.width + blueJewel.size.height) / 4.0;
                    double meanRadius = (redRadius + blueRadius) / 2.0;
                    double distance = Math.hypot(redJewel.center.x - blueJewel.center.x, redJewel.center.y - blueJewel.center.y);
                    double distanceRatio = distance / meanRadius;
                    double distanceRatioError = Math.abs(distanceRatio - DIST_RATIO);
                    if (distanceRatioError < minDistRatioError) {
                        minDistRatioError = distanceRatioError;
                        bestRedJewel = redJewel;
                        bestBlueJewel = blueJewel;
                    }
                }
            }
            if (minDistRatioError > MAX_DIST_RATIO_ERROR) {
                leftJewelColor = JewelColor.UNKNOWN;
            } else {
                leftJewelColor = bestRedJewel.center.x < bestBlueJewel.center.x ? JewelColor.RED : JewelColor.BLUE;
            }
        }
    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        overlay.setScalingFactor(4);

        if (lastRedJewels != null) {
            for (MatOfPoint contour : lastContours) {
                overlay.strokeContour(contour, new Scalar(255, 255, 255), 10);
            }

            for (RotatedRect redJewel : lastRedJewels) {
                overlay.strokeCircle(redJewel.center, (redJewel.size.width + redJewel.size.height) / 4, new Scalar(0, 0, 255), 10);
            }

            for (RotatedRect blueJewel : lastBlueJewels) {
                overlay.strokeCircle(blueJewel.center, (blueJewel.size.width + blueJewel.size.height) / 4, new Scalar(255, 0, 0), 10);
            }
        }

        overlay.setScalingFactor(1);

        overlay.putText(
                toString(),
                Overlay.TextAlign.LEFT,
                new Point(5, 50),
                new Scalar(0, 0, 255),
                45
        );
    }

    public synchronized JewelColor getLeftColor() {
        return leftJewelColor;
    }

    public synchronized JewelColor getRightColor() {
        return leftJewelColor.opposite();
    }

    @Override
    public synchronized String toString() {
        return leftJewelColor + " / " + leftJewelColor.opposite();
    }
}
