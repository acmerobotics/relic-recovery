package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

@Config
public class FixedJewelTracker extends Tracker {
    public static final String TAG = "FixedJewelTracker";

    public static Point LEFT_CENTER = new Point(825, 500);
    public static Point RIGHT_CENTER = new Point(1200, 500);

    public static int LEFT_RADIUS = 125;
    public static int RIGHT_RADIUS = 125;

    public static double COLOR_THRESHOLD = 0.7;

    private double leftRed, leftBlue, rightRed, rightBlue;
    private Mat leftMask, rightMask, left, right;

    @Override
    public void init(VisionCamera camera) {
        left = new Mat();
        right = new Mat();
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        int imageWidth = frame.width(), imageHeight = frame.height();

        if (leftMask == null) {
            leftMask = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
            rightMask = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
        }

        leftMask.setTo(new Scalar(0, 0, 0));
        rightMask.setTo(new Scalar(0, 0, 0));

        Imgproc.circle(leftMask, LEFT_CENTER, LEFT_RADIUS, new Scalar(255, 255, 255), Core.FILLED);
        Imgproc.circle(rightMask, RIGHT_CENTER, RIGHT_RADIUS, new Scalar(255, 255, 255), Core.FILLED);

        Core.bitwise_and(leftMask, frame, left);
        Core.bitwise_and(rightMask, frame, right);

        Scalar leftJewelTotalColor = Core.sumElems(left);
        Scalar rightJewelTotalColor = Core.sumElems(right);

        leftBlue = leftJewelTotalColor.val[0];
        leftRed = leftJewelTotalColor.val[2];
        rightBlue = rightJewelTotalColor.val[0];
        rightRed = rightJewelTotalColor.val[2];

        double leftTotal = leftBlue + leftRed;
        double rightTotal = rightBlue + rightRed;

        if (leftTotal == 0) {
            leftBlue = 0;
            leftRed = 0;
        } else {
            leftBlue /= leftTotal;
            leftRed /= leftTotal;
        }

        if (rightTotal == 0) {
            rightBlue = 0;
            rightRed = 0;
        } else {
            rightBlue /= rightTotal;
            rightRed /= rightTotal;
        }
    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        Scalar leftColor;
        if (leftBlue > COLOR_THRESHOLD) {
            leftColor = new Scalar(255, 0, 0);
        } else if (leftRed > COLOR_THRESHOLD) {
            leftColor = new Scalar(0, 0, 255);
        } else {
            leftColor = new Scalar(0, 255, 0);
        }
        overlay.strokeCircle(LEFT_CENTER, LEFT_RADIUS, leftColor, 5);

        Scalar rightColor;
        if (rightBlue > COLOR_THRESHOLD) {
            rightColor = new Scalar(255, 0, 0);
        } else if (rightRed > COLOR_THRESHOLD) {
            rightColor = new Scalar(0, 0, 255);
        } else {
            rightColor = new Scalar(0, 255, 0);
        }
        overlay.strokeCircle(RIGHT_CENTER, RIGHT_RADIUS, rightColor, 5);
    }


    public synchronized double getLeftBlue() {
        return leftBlue;
    }

    public synchronized double getLeftRed() {
        return leftRed;
    }

    public synchronized double getRightBlue() {
        return rightBlue;
    }

    public synchronized double getRightRed() {
        return rightRed;
    }

    public synchronized JewelPosition getJewelPosition() {
        return leftBlue > rightBlue ? JewelPosition.BLUE_RED : JewelPosition.RED_BLUE;
    }
}
