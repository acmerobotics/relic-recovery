package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.splinelib.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "Demo", group = "teleop")
public class Demo extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private Robot robot;

    private boolean intakeRunning, relicModeActive;
    private int leftIntakePower, rightIntakePower;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        telemetry.setMsTransmissionInterval(50);
        telemetry.addLine("Ready");
    }

    @Override
    public void start() {
        robot.intake.releaseBar();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        // drive
        double x, y = 0, omega;

        x = -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_x) > 0.5) {
            y = -gamepad1.left_stick_x;
        }

        omega = -gamepad1.right_stick_x;

        x *= 0.5;
        y *= 0.5;
        omega *= 0.5;

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        robot.drive.setVelocity(new Vector2d(x, y), omega);

        // relic mode
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

            if (stickyGamepad2.x) {
                robot.relicRecoverer.setArmPosition(RelicRecoverer.MAX_EXTENSION_DISTANCE);
            } else if (stickyGamepad2.a) {
                robot.relicRecoverer.setArmPosition(0);
            } else if (gamepad2.left_stick_y != 0 || robot.relicRecoverer.getArmMode() == RelicRecoverer.ArmMode.MANUAL) {
                robot.relicRecoverer.setArmPower(-gamepad2.left_stick_y);
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
            }

            // dump bed
            if (stickyGamepad2.right_bumper) {
                if (robot.dumpBed.isDumping()) {
                    robot.dumpBed.retract();
                } else {
                    robot.dumpBed.dump();
                }
            }

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
    }
}

