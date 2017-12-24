package com.acmerobotics.relicrecovery.vision;

import org.opencv.core.Mat;

/**
 * Created by ryanbrott on 9/23/17.
 */

public abstract class Tracker {
    private boolean enabled = true;

    void internalProcessFrame(Mat frame, double timestamp) {
        if (enabled) {
            processFrame(frame, timestamp);
        }
    }

    public void enable() {
        enabled = true;
    }

    public void disable() {
        enabled = false;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public abstract void init(VisionCamera camera);
    public abstract void processFrame(Mat frame, double timestamp);
    public abstract void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug);
}
