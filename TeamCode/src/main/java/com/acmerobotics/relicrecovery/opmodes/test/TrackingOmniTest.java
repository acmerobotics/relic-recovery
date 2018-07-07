package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.localization.TrackingOmniLocalizer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.splinelib.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TrackingOmniTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        robot.drive.setLocalizer(new TrackingOmniLocalizer(robot.drive));
        robot.drive.enablePositionEstimation();
        robot.start();

        waitForStart();

        while (opModeIsActive()) {
            double x, y = 0, omega;

            x = -gamepad1.left_stick_y;

            if (Math.abs(gamepad1.left_stick_x) > 0.5) {
                y = -gamepad1.left_stick_x;
            }

            omega = -gamepad1.right_stick_x;

            if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
                y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
            }

            robot.drive.setVelocity(new Vector2d(x, y), omega);
        }
    }
}
