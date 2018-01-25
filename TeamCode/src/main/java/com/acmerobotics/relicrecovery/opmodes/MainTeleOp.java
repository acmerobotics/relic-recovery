package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private Robot robot;

    private boolean halfSpeed, intakeRunning;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.setMsTransmissionInterval(50);
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        // drive
        if (stickyGamepad1.b) {
            halfSpeed = !halfSpeed;
        }

        double x, y = 0, omega;

        x = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_x) > 0.5) {
            y = -gamepad1.left_stick_x;
        }

        omega = -gamepad1.right_stick_x;

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("omega", omega);

        robot.drive.setVelocity(new Vector2d(x, y), omega);

        // dump bed
        if (stickyGamepad1.dpad_up) {
            robot.dumpBed.moveUp();
        }

        if (stickyGamepad1.dpad_down) {
            robot.dumpBed.moveDown();
        }

        if (stickyGamepad1.right_bumper) {
            if (robot.dumpBed.isDumping()) {
                robot.dumpBed.retract();
            } else {
                robot.dumpBed.dump();
            }
        }

        // intake
        if (stickyGamepad1.left_bumper) {
            if (intakeRunning) {
                robot.intake.setIntakePower(0);
                intakeRunning = false;
            } else if (!robot.dumpBed.isDumping() && !robot.dumpBed.isLiftUp()) {
                robot.intake.setIntakePower(1);
                intakeRunning = true;
            }
        }

        telemetry.addData("x", Math.random());
    }
}

