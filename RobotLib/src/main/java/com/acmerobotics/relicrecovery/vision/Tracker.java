package com.acmerobotics.relicrecovery.vision;

import org.opencv.core.Mat;

import java.util.Collections;
import java.util.List;

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

    public List<LabeledMat> getIntermediates() {
        return Collections.emptyList();
    }

    public abstract void init(VisionCamera camera);
    public abstract void processFrame(Mat frame, double timestamp);
    public abstract void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug);
}
