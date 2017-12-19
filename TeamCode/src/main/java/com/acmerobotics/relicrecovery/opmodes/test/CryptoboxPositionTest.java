package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.configuration.Cryptobox;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author Ryan
 */

@TeleOp
public class CryptoboxPositionTest extends OpMode {
    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;
    private FpsTracker fpsTracker;
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        camera = new VisionCamera();
        cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.setImageDir(LoggingUtil.getImageDir(this));
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);
    }

    @Override
    public void loop() {
        Vector2d pos = CryptoboxTracker.getFieldPositionFromCryptoRelativePosition(
                Cryptobox.NEAR_BLUE, cryptoboxTracker.getLatestPositionEstimate().data);
        if (!Double.isNaN(pos.x()) && !Double.isNaN(pos.y())) {
            fieldOverlay.setFill("blue");
            fieldOverlay.fillCircle(pos.x(), pos.y(), 2);
            dashboard.drawOverlay();
        }
        telemetry.addData("x", pos.x());
        telemetry.addData("y", pos.y());
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
