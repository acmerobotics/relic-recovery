package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Pose2d;
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
        cryptoboxTracker = new CryptoboxTracker(CryptoboxTracker.Color.BLUE);
        fpsTracker = new FpsTracker();
        camera.addTracker(cryptoboxTracker);
        camera.addTracker(fpsTracker);
        camera.setImageDir(LoggingUtil.getImageDir(this));
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);
    }

    @Override
    public void loop() {
        CryptoboxTracker.Result result = cryptoboxTracker.getLatestResult();
        Pose2d cryptobox = new Pose2d(12, 48);
        Pose2d robot = cryptobox.added(new Pose2d(result.offsetX, -result.distance, 0));
        fieldOverlay.setFill("blue");
        fieldOverlay.fillCircle(robot.x(), robot.y(), 5);
        dashboard.drawOverlay();
        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

}
