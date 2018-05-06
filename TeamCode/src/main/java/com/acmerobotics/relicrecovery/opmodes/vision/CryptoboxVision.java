package com.acmerobotics.relicrecovery.opmodes.vision;

import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.library.vision.FpsTracker;
import com.acmerobotics.library.vision.OpenCVCamera;
import com.acmerobotics.library.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp(name = "Cryptobox Vision", group = "vision")
public class CryptoboxVision extends OpMode {
    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;

    @Override
    public void init() {
        camera = new OpenCVCamera();
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
