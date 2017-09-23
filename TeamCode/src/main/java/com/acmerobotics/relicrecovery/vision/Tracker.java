package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;

import org.opencv.core.Mat;

/**
 * Created by ryanbrott on 9/23/17.
 */

public interface Tracker {
    void processFrame(Mat frame);
    void drawOverlay(Canvas canvas, int imageWidth, int imageHeight);
}
