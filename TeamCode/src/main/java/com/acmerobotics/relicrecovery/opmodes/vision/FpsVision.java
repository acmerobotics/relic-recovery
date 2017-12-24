package com.acmerobotics.relicrecovery.opmodes.vision;

import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 9/23/17.
 */
@TeleOp(name = "FPS Vision", group = "vision")
public class FpsVision extends OpMode {
    private VuforiaCamera camera;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new VuforiaCamera(VisionConstants.VUFORIA_PARAMETERS);
        fpsTracker = new FpsTracker();
        camera.addTracker(fpsTracker);
        camera.initialize();
    }

    @Override
    public void loop() {

    }
}
