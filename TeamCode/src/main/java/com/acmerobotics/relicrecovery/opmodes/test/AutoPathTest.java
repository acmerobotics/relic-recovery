package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.library.util.DrawingUtil;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.splinelib.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Disabled
@TeleOp
public class AutoPathTest extends OpMode {
    private RobotDashboard dashboard;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
    }

    @Override
    public void loop() {
        TelemetryPacket telemetryPacket = new TelemetryPacket();
        Canvas fieldOverlay = telemetryPacket.fieldOverlay();
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
        dashboard.sendTelemetryPacket(telemetryPacket);
    }
}
