package com.acmerobotics.relicrecovery.opmodes.vision;

import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 9/23/17.
 */

@TeleOp(name = "Cryptobox Vision", group = "vision")
public class CryptoboxVision extends OpMode {
    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new VisionCamera();
        cryptoboxTracker = new CryptoboxTracker(false);
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);
    }

    @Override
    public void loop() {

    }
}
