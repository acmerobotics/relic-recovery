package com.acmerobotics.relicrecovery.opmodes.vision;

import com.acmerobotics.relicrecovery.vision.FeatureDetectionVuMarkTracker;
import com.acmerobotics.library.vision.FpsTracker;
import com.acmerobotics.library.vision.OpenCVCamera;
import com.acmerobotics.library.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Feature Detection Vision", group = "vision")
public class FeatureDetectionVision extends OpMode {
    private VisionCamera camera;
    private FeatureDetectionVuMarkTracker vuMarkTracker;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new OpenCVCamera();
        vuMarkTracker = new FeatureDetectionVuMarkTracker();
        fpsTracker = new FpsTracker();
        camera.addTracker(vuMarkTracker);
        camera.addTracker(fpsTracker);
        camera.initialize();
    }

    @Override
    public void loop() {
        telemetry.addData("vuMark", vuMarkTracker.getVuMark());
    }
}
