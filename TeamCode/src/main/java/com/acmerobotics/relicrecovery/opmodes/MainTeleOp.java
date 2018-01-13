package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.subsystems.Intake;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private RobotDashboard dashboard;

    private MecanumDrive drive;
    private DumpBed dumpBed;
    private JewelSlapper jewelSlapper;
    private Intake intake;
    private RelicRecoverer relicRecoverer;

    private boolean halfSpeed;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        dashboard = RobotDashboard.getInstance();

        drive = new MecanumDrive(hardwareMap);
        dumpBed = new DumpBed(hardwareMap, dashboard.getTelemetry());
        jewelSlapper = new JewelSlapper(hardwareMap);
        intake = new Intake(hardwareMap, dashboard.getTelemetry());
        relicRecoverer = new RelicRecoverer(hardwareMap);

        dumpBed.liftDown();
    }

    @Override
    public void init_loop() {
        dumpBed.update();
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

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        omega = -gamepad1.right_stick_x;

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        drive.setVelocity(new Vector2d(x, y), omega);

//        if (drive.getMode() == MecanumDrive.Mode.OPEN_LOOP || drive.getMode() == MecanumDrive.Mode.OPEN_LOOP_RAMP) {
//            drive.setVelocity(new Vector2d(x, y), omega);
//        } else if (x != 0 && y != 0 && omega != 0) {
//            drive.setVelocity(new Vector2d(x, y), omega);
//        }

        // dump bed
        if (stickyGamepad1.left_bumper) {
            if (dumpBed.isLiftDown()) {
                dumpBed.liftUp();
            } else {
                dumpBed.liftDown();
            }
        }

        if (stickyGamepad1.right_bumper) {
            if (dumpBed.isDumping()) {
                dumpBed.retract();
            } else {
                dumpBed.dump();
            }
        }

        // intake
        if (stickyGamepad2.right_bumper) {
            if (intake.isRotatedDown()) {
                intake.rotateUp();
            } else {
                intake.rotateDown();
            }
        }

        if (stickyGamepad2.left_bumper) {
            if (intake.isClosed()) {
                intake.open();
            } else {
                intake.close();
            }
        }

        if (gamepad2.left_trigger > 0.8) {
            intake.engageFlipper();
        } else {
            intake.disengageFlipper();
        }

        // relic
        if (gamepad2.right_stick_y != 0) {
            relicRecoverer.setExtendPower(-0.25 * gamepad2.left_stick_y);
        } else {
            relicRecoverer.setExtendPower(0);
        }

        if (gamepad2.x) {
            relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.VERTICAL);
        }

        if (gamepad2.y) {
            relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.HORIZONTAL);
        }

        if (gamepad2.a) {
            relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.STOW);
        }

        if (gamepad2.b) {
            if (relicRecoverer.isFingerClosed()) {
                relicRecoverer.openFinger();
            } else {
                relicRecoverer.closeFinger();
            }
        }

        drive.update();
        dumpBed.update();
        intake.update();

        dashboard.getTelemetry().update();
    }
}

