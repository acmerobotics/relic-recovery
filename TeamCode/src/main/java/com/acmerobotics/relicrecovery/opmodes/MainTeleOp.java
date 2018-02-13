package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private Robot robot;

    private boolean intakeRunning, relicModeActive, slowMode, superSlowMode; // , maintainHeading;
    private int leftIntakePower, rightIntakePower;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);
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
            slowMode = !slowMode;
            superSlowMode = false;
        } else if (stickyGamepad1.left_bumper) {
            superSlowMode = !superSlowMode;
            slowMode = false;
        }

        double x, y = 0, omega;

        x = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_x) > 0.5) {
            y = -gamepad1.left_stick_x;
        }

        omega = -gamepad1.right_stick_x;

        if (superSlowMode) {
            x *= 0.25;
            y *= 0.25;
            omega *= 0.25;
        } else if (slowMode) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        robot.drive.setVelocity(new Vector2d(x, y), omega);

        if (stickyGamepad1.right_bumper) {
            if (robot.dumpBed.isDumping()) {
                robot.dumpBed.retract();
            } else {
                robot.dumpBed.dump();
            }
        }

//        if (stickyGamepad1.a) {
//            maintainHeading = !maintainHeading;
//            if (maintainHeading) {
//                robot.drive.enableHeadingCorrection();
//            } else {
//                robot.drive.disableHeadingCorrection();
//            }
//        }

        if (stickyGamepad2.right_stick_button) {
            relicModeActive = !relicModeActive;
        }

        if (relicModeActive) {
            // relic
            if (stickyGamepad2.dpad_up) {
                robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);
            } else if (stickyGamepad2.dpad_down) {
                robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.DOWN);
            }

            if (stickyGamepad2.y) {
                robot.relicRecoverer.openFinger();
            } else if (stickyGamepad2.b) {
                robot.relicRecoverer.closeFinger();
            }

            if (gamepad2.left_stick_y != 0) {
                robot.relicRecoverer.setArmPower(-gamepad2.left_stick_y);
            } else if (stickyGamepad2.x) {
                robot.relicRecoverer.setArmPosition(RelicRecoverer.MAX_EXTENSION_DISTANCE);
            } else if (stickyGamepad2.a) {
                robot.relicRecoverer.setArmPosition(0);
            }

            // intake
            robot.intake.setIntakePower(0);
        } else {
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

            // dump bed
            if (gamepad2.y) {
                robot.dumpBed.setLiftPower(DumpBed.LIFT_UP_POWER);
            } else if (gamepad2.x) {
                robot.dumpBed.setLiftPower(DumpBed.LIFT_DOWN_POWER);
            } else if (stickyGamepad2.dpad_up) {
                robot.dumpBed.moveUp();
            } else if (stickyGamepad2.dpad_down) {
                robot.dumpBed.moveDown();
            } else if (robot.dumpBed.getLiftMode() == DumpBed.LiftMode.MANUAL) {
                robot.dumpBed.setLiftPower(0);
            }

            if (robot.dumpBed.isDumping() || robot.dumpBed.isLiftUp() || robot.dumpBed.getLiftMode() != DumpBed.LiftMode.MANUAL) {
                robot.intake.setIntakePower(0, 0);
            } else if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                robot.intake.setIntakePower(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
            } else {
                robot.intake.setIntakePower(leftIntakePower, rightIntakePower);
            }

            // relic
            robot.relicRecoverer.setArmPower(0);
        }

        telemetry.addData("relicModeActive", relicModeActive);
//        telemetry.addData("maintainHeading", maintainHeading);
    }
}

