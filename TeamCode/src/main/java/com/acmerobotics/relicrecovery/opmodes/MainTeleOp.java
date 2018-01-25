package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private Robot robot;

    private boolean halfSpeed, intakeRunning;
    private int leftIntakePower, rightIntakePower;

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
        if (stickyGamepad1.x) {
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
        if (stickyGamepad1.b) {
            if (robot.dumpBed.isLiftUp()) {
                robot.dumpBed.moveDown();
            } else {
                robot.dumpBed.moveUp();
            }
        }

        if (stickyGamepad1.right_bumper) {
            if (robot.dumpBed.isDumping()) {
                robot.dumpBed.retract();
            } else {
                robot.dumpBed.dump();
            }
        }

        // intake
        if (stickyGamepad2.left_bumper) {
            if (intakeRunning) {
                leftIntakePower = 0;
                rightIntakePower = 0;
                intakeRunning = false;
            } else {
                leftIntakePower = 1;
                rightIntakePower = 1;
                intakeRunning = true;
            }
        } else if (stickyGamepad2.right_bumper) {
            if (intakeRunning) {
                leftIntakePower = 0;
                rightIntakePower = 0;
                intakeRunning = false;
            } else {
                leftIntakePower = -1;
                rightIntakePower = -1;
                intakeRunning = true;
            }
        }

        if (robot.dumpBed.isDumping() || robot.dumpBed.isLiftUp() || robot.dumpBed.getMode() != DumpBed.Mode.STATIC) {
            robot.intake.setIntakePower(0, 0);
        } else if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
            robot.intake.setIntakePower(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
        } else {
            robot.intake.setIntakePower(leftIntakePower, rightIntakePower);
        }

        telemetry.addData("x", Math.random());
    }
}

