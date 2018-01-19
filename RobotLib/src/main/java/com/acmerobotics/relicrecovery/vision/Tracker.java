package com.acmerobotics.relicrecovery.vision;

import org.opencv.core.Mat;

import java.util.LinkedHashMap;

public abstract class Tracker {
    private boolean enabled = true;
    private LinkedHashMap<String, Mat> intermediates = new LinkedHashMap<>();

    void internalProcessFrame(Mat frame, double timestamp) {
        if (enabled) {
            intermediates.clear();
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

    public LinkedHashMap<String, Mat> getIntermediates() {
        return intermediates;
    }

    protected void addIntermediate(String key, Mat value) {
        intermediates.put(key, value);
    }

    public abstract void init(VisionCamera camera);
    public abstract void processFrame(Mat frame, double timestamp);
    public abstract void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug);
}
