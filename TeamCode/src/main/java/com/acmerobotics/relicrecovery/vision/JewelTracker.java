package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class JewelTracker implements Tracker {
    public static final String TAG = "JewelTracker";

    public static final float JEWEL_PLATFORM_ASPECT_RATIO = 2.6f; // width/height
    public static final double COLOR_THRESHOLD = 0.7;

    private Paint paint;
    private Rect jewelPlatformRect, leftJewelRect, rightJewelRect;
    private double leftRed, leftBlue, rightRed, rightBlue;

    public JewelTracker() {
        paint = new Paint();
        paint.setStyle(Paint.Style.STROKE);
        paint.setStrokeWidth(5.0f);
    }

    @Override
    public synchronized void processFrame(Mat frame) {
        if (jewelPlatformRect == null) {
            int imageWidth = frame.width(), imageHeight = frame.height();
            int platformWidth = imageWidth / 2;
            int platformHeight = (int) (platformWidth / JEWEL_PLATFORM_ASPECT_RATIO);
            int offsetX = (imageWidth - platformWidth) / 2;
            int offsetY = (imageHeight - platformHeight) / 2;

            jewelPlatformRect = new Rect(offsetX, offsetY, platformWidth, platformHeight);
            leftJewelRect = new Rect(offsetX, offsetY, platformHeight, platformHeight);
            rightJewelRect = new Rect(offsetX + platformWidth - platformHeight, offsetY, platformHeight, platformHeight);
        }

        Mat leftJewelCropped = frame.submat(leftJewelRect);
        Mat rightJewelCropped = frame.submat(rightJewelRect);

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
    public synchronized void drawOverlay(Canvas canvas, int imageWidth, int imageHeight) {
        if (jewelPlatformRect != null) {
            paint.setColor(Color.GREEN);
            VisionUtil.drawRect(canvas, jewelPlatformRect, paint);

            if (leftBlue > COLOR_THRESHOLD) {
                paint.setColor(Color.BLUE);
            } else if (leftRed > COLOR_THRESHOLD) {
                paint.setColor(Color.RED);
            } else {
                paint.setColor(Color.GREEN);
            }
            VisionUtil.drawRect(canvas, leftJewelRect, paint);

            if (rightBlue > COLOR_THRESHOLD) {
                paint.setColor(Color.BLUE);
            } else if (rightRed > COLOR_THRESHOLD) {
                paint.setColor(Color.RED);
            } else {
                paint.setColor(Color.GREEN);
            }
            VisionUtil.drawRect(canvas, rightJewelRect, paint);
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
