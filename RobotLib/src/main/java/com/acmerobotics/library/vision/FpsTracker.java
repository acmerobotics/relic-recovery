package com.acmerobotics.library.vision;

import com.acmerobotics.library.util.ExponentialSmoother;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;

import java.util.Locale;

public class FpsTracker extends Tracker {
    private ExponentialSmoother smoother;
    private double lastTimestamp, avgTimeDelta;

    public FpsTracker() {
        smoother = new ExponentialSmoother(0.2);
    }

    @Override
    public void init(VisionCamera camera) {

    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        if (lastTimestamp != 0) {
            double timeDelta = timestamp - lastTimestamp;
            avgTimeDelta = smoother.update(timeDelta);
        }
        lastTimestamp = timestamp;
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        overlay.putText(String.format(Locale.ENGLISH, "%.2f FPS", 1 / avgTimeDelta), Overlay.TextAlign.RIGHT, new Point(imageWidth - 5, 45), new Scalar(0, 0, 255), 45);
    }
}
