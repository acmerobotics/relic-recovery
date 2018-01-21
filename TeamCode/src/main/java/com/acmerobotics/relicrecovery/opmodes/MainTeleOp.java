package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private Robot robot;

    private boolean halfSpeed;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.setMsTransmissionInterval(50);

//        dumpBed.liftDown();
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

//        if (drive.getMode() == MecanumDrive.Mode.OPEN_LOOP || drive.getMode() == MecanumDrive.Mode.OPEN_LOOP_RAMP) {
//            drive.setVelocity(new Vector2d(x, y), omega);
//        } else if (x != 0 && y != 0 && omega != 0) {
//            drive.setVelocity(new Vector2d(x, y), omega);
//        }

//        // dump bed
//        if (stickyGamepad1.left_bumper) {
//            if (dumpBed.isLiftDown()) {
//                dumpBed.liftUp();
//            } else {
//                dumpBed.liftDown();
//            }
//        }
//
//        if (stickyGamepad1.right_bumper) {
//            if (dumpBed.isDumping()) {
//                dumpBed.retract();
//            } else {
//                dumpBed.dump();
//            }
//        }
//
//        // intake
//        if (stickyGamepad2.left_bumper) {
//            if (intake.isClosed()) {
//                intake.open();
//            } else {
//                intake.close();
//            }
//        }
//
//        if (gamepad2.left_trigger > 0.8) {
//            intake.rotateUp();
//        } else {
//            intake.rotateDown();
//        }
//
//        if (gamepad2.right_trigger > 0.8) {
//            intake.engageFlipper();
//        } else {
//            intake.disengageFlipper();
//        }
//
//        // relic
//        if (gamepad2.right_stick_y != 0) {
//            relicRecoverer.setExtendPower(-0.25 * gamepad2.left_stick_y);
//        } else {
//            relicRecoverer.setExtendPower(0);
//        }
//
//        if (gamepad2.x) {
//            relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.VERTICAL);
//        }
//
//        if (gamepad2.y) {
//            relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.HORIZONTAL);
//        }
//
//        if (gamepad2.a) {
//            relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.STOW);
//        }
//
//        if (gamepad2.b) {
//            if (relicRecoverer.isFingerClosed()) {
//                relicRecoverer.openFinger();
//            } else {
//                relicRecoverer.closeFinger();
//            }
//        }

        telemetry.addData("leftStickX", gamepad1.left_stick_x);
        telemetry.addData("leftStickY", gamepad1.left_stick_y);
        telemetry.addData("rightStickX", gamepad1.right_stick_x);
        telemetry.addData("rightStickY", gamepad1.right_stick_y);
    }
}

