package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Locale;

/**
 * Created by ryanbrott on 10/28/17.
 */

@Disabled
@TeleOp(name = "Pose Estimation Test", group = "test")
public class PoseEstimationTest extends LinearOpMode {
    private Looper looper;

    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    private MecanumDrive drive;

    @Override
    public void runOpMode() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        looper = new Looper(20);

        drive = new MecanumDrive(hardwareMap);
        drive.registerLoops(looper);

        waitForStart();

        looper.start();

        while (opModeIsActive()) {
            if (gamepad1.a) {
                drive.setEstimatedPose(new Pose2d(0, 0, 0));
            }

            drive.setVelocity(new Vector2d(
                    -gamepad1.left_stick_y,
                    -gamepad1.left_stick_x
            ), -gamepad1.right_stick_y);

            Pose2d estimatedPose = drive.getEstimatedPose();

            fieldOverlay.setFill("green");
            fieldOverlay.fillCircle(estimatedPose.x(), estimatedPose.y(), 5);
            dashboard.drawOverlay();

            telemetry.addData(">", "Press [A] to reset pose to (0, 0, 0)");
            telemetry.addData("x", estimatedPose.x());
            telemetry.addData("y", estimatedPose.y());
            telemetry.addData("heading", String.format(Locale.ENGLISH, "%.2f (%.2fdeg)", estimatedPose.heading(), Math.toDegrees(estimatedPose.heading())));
            int[] pos = drive.getPositions();
            for (int i = 0; i < pos.length; i++) {
                telemetry.addData("encoder" + i, pos[i]);
            }
            telemetry.update();

            sleep(20);
        }
    }
}
