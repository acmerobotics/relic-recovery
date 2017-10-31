package com.acmerobotics.relicrecovery.vision;

import org.opencv.core.Mat;

/**
 * Created by ryanbrott on 9/23/17.
 */

public interface Tracker {
    void processFrame(Mat frame, double timestamp);
    void drawOverlay(Overlay overlay, int imageWidth, int imageHeight);
}
