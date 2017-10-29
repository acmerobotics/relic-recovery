package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.drive.PoseEstimator;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Created by ryanbrott on 10/28/17.
 */

@TeleOp(name = "Pose Estimation Test")
public class PoseEstimationTest extends OpMode {
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    private Looper looper;
    private MecanumDrive drive;
    private PoseEstimator poseEstimator;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        looper = new Looper(20);
        drive = new MecanumDrive(hardwareMap);
        poseEstimator = new PoseEstimator(drive, new Pose2d(0, 0, 0));
        looper.addLoop(poseEstimator);
        looper.start();
    }

    @Override
    public void loop() {
        if (gamepad1.a) {
            poseEstimator.setPose(new Pose2d(0, 0, 0));
        }

        drive.setVelocity(new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ), -gamepad1.right_stick_y);

        Pose2d pose = poseEstimator.getPose();

        fieldOverlay.setFill("green");
        fieldOverlay.fillCircle(pose.x(), pose.y(), 5);
        dashboard.drawOverlay();

        telemetry.addData(">", "Press [A] to reset pose to (0, 0, 0)");
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("heading", String.format(Locale.ENGLISH, "%.2f (%.2fdeg)", pose.heading(), Math.toDegrees(pose.heading())));
    }
}
