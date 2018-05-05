package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.library.util.DrawingUtil;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Disabled
@TeleOp
public class AutoPathTest extends OpMode {
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
    }

    @Override
    public void loop() {
        fieldOverlay.setStroke("#4CAF50");
        DrawingUtil.drawTrajectory(fieldOverlay, AutoPaths.makeDiagonalTrajectoryToCryptobox(
                BalancingStone.FAR_BLUE, RelicRecoveryVuMark.LEFT));
        DrawingUtil.drawTrajectory(fieldOverlay, AutoPaths.makeDiagonalTrajectoryToCryptobox(
                BalancingStone.FAR_BLUE, RelicRecoveryVuMark.CENTER));
        DrawingUtil.drawTrajectory(fieldOverlay, AutoPaths.makeDiagonalTrajectoryToCryptobox(
                BalancingStone.FAR_BLUE, RelicRecoveryVuMark.RIGHT));
        fieldOverlay.setStroke("#F44336");
        DrawingUtil.drawMecanumRobot(fieldOverlay, new Pose2d(
                BalancingStone.NEAR_BLUE.getPosition(), 0));
        dashboard.drawOverlay();
    }
}
