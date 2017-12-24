package com.acmerobotics.relicrecovery.opmodes.vision;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 9/23/17.
 */

@TeleOp(name = "Cryptobox Vision", group = "vision")
public class CryptoboxVision extends OpMode {
    private VuforiaCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new VuforiaCamera(VisionConstants.VUFORIA_PARAMETERS);
        cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.initialize();
    }

    @Override
    public void loop() {

    }
}
