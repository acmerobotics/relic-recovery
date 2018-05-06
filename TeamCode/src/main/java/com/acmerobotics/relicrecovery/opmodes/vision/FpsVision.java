package com.acmerobotics.relicrecovery.opmodes.vision;

import android.graphics.Bitmap;

import com.acmerobotics.library.cameraoverlay.CameraStreamServer;
import com.acmerobotics.library.vision.FpsTracker;
import com.acmerobotics.library.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.core.Mat;

/**
 * Created by ryanbrott on 9/23/17.
 */
@Disabled
@TeleOp(name = "FPS Vision", group = "vision")
public class FpsVision extends OpMode {
    private VuforiaCamera camera;
    private FpsTracker fpsTracker;
    private CameraStreamServer streamServer;
    private Bitmap bitmap;
    private Mat rgba;

    @Override
    public void init() {
        streamServer = new CameraStreamServer();
        camera = new VuforiaCamera();
        fpsTracker = new FpsTracker();
        camera.addTracker(fpsTracker);
        camera.addTracker(streamServer.getTracker());
        camera.initialize();
    }

    @Override
    public void start() {
        streamServer.stop();
    }

    @Override
    public void loop() {

    }
}
