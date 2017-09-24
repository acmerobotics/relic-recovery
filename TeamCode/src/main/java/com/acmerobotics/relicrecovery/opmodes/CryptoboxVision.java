package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 9/23/17.
 */
@TeleOp
public class CryptoboxVision extends OpMode {
    private VisionCamera camera;
    private CryptoboxTracker tracker;

    @Override
    public void init() {
        camera = new VisionCamera(hardwareMap.appContext);
        tracker = new CryptoboxTracker();
        camera.addTracker(tracker);
        camera.initialize();
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        camera.close();
    }
}
