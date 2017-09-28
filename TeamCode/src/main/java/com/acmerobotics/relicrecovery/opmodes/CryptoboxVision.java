package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.acmerobotics.relicrecovery.vision.VisionUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.ClassFactory;

/**
 * Created by ryanbrott on 9/23/17.
 */
@TeleOp
public class CryptoboxVision extends OpMode {
    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new VisionCamera(hardwareMap.appContext, ClassFactory.createVuforiaLocalizer(VisionConstants.VUFORIA_PARAMETERS));
        camera.setImageDir(VisionUtil.getImageDir(this));
        cryptoboxTracker = new CryptoboxTracker();
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
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
