package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.config.Config;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Rect2d;
import org.opencv.core.Scalar;

/**
 * Created by ryanbrott on 9/23/17.
 */

@Config
public class FixedJewelTracker extends Tracker {
    public static final String TAG = "FixedJewelTracker";

    public static final float JEWEL_PLATFORM_ASPECT_RATIO = 2.6f; // width/height

    public static Rect2d LEFT_JEWEL_RECT = new Rect2d(0.5 - JEWEL_PLATFORM_ASPECT_RATIO * 0.125, 0.375, 0.25, 0.25);
    public static Rect2d RIGHT_JEWEL_RECT = new Rect2d(0.25 + JEWEL_PLATFORM_ASPECT_RATIO * 0.125, 0.375, 0.25, 0.25);

    public static double COLOR_THRESHOLD = 0.7;

    private Rect scaledLeftJewelRect, scaledRightJewelRect;
    private double leftRed, leftBlue, rightRed, rightBlue;

    @Override
    public void init(VisionCamera camera) {

    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        int imageWidth = frame.width(), imageHeight = frame.height();
        scaledLeftJewelRect = new Rect(
                (int) (imageWidth * LEFT_JEWEL_RECT.x),
                (int) (imageHeight * LEFT_JEWEL_RECT.y),
                (int) (imageWidth * LEFT_JEWEL_RECT.width),
                (int) (imageHeight * LEFT_JEWEL_RECT.height)
        );
        scaledRightJewelRect = new Rect(
                (int) (imageWidth * RIGHT_JEWEL_RECT.x),
                (int) (imageHeight * RIGHT_JEWEL_RECT.y),
                (int) (imageWidth * RIGHT_JEWEL_RECT.width),
                (int) (imageHeight * RIGHT_JEWEL_RECT.height)
        );

        Mat leftJewelCropped = frame.submat(scaledLeftJewelRect);
        Mat rightJewelCropped = frame.submat(scaledRightJewelRect);

        Scalar leftJewelTotalColor = Core.sumElems(leftJewelCropped);
        Scalar rightJewelTotalColor = Core.sumElems(rightJewelCropped);

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
        if (scaledLeftJewelRect != null) {
            Scalar leftColor;
            if (leftBlue > COLOR_THRESHOLD) {
                leftColor = new Scalar(255, 0, 0);
            } else if (leftRed > COLOR_THRESHOLD) {
                leftColor = new Scalar(0, 0, 255);
            } else {
                leftColor = new Scalar(0, 255, 0);
            }
            overlay.strokeRect(scaledLeftJewelRect, leftColor, 5);

            Scalar rightColor;
            if (rightBlue > COLOR_THRESHOLD) {
                rightColor = new Scalar(255, 0, 0);
            } else if (rightRed > COLOR_THRESHOLD) {
                rightColor = new Scalar(0, 0, 255);
            } else {
                rightColor = new Scalar(0, 255, 0);
            }
            overlay.strokeRect(scaledRightJewelRect, rightColor, 5);
        }
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
}
