package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 9/23/17.
 */
@TeleOp
public class FpsVision extends OpMode {
    private VisionCamera camera;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new VisionCamera(hardwareMap.appContext);
        fpsTracker = new FpsTracker();
        camera.addTracker(fpsTracker);
        camera.setImageDir(LoggingUtil.getImageDir(this));
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);
    }

    @Override
    public void loop() {

    }

    @Override
    public void stop() {
        camera.close();
    }
}
