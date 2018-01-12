package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.subsystems.Intake;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.PhoneSwivel;
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
    private PhoneSwivel swivel;

    private boolean halfSpeed;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        dashboard = RobotDashboard.getInstance();

        drive = new MecanumDrive(hardwareMap);
        dumpBed = new DumpBed(hardwareMap, telemetry);
        jewelSlapper = new JewelSlapper(hardwareMap);
//        intake = new Intake(hardwareMap);
        swivel = new PhoneSwivel(hardwareMap);

        swivel.pointAtJewel();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

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

        if (drive.getMode() == MecanumDrive.Mode.OPEN_LOOP || drive.getMode() == MecanumDrive.Mode.OPEN_LOOP_RAMP) {
            drive.setVelocity(new Vector2d(x, y), omega);
        } else if (x != 0 && y != 0 && omega != 0) {
            drive.setVelocity(new Vector2d(x, y), omega);
        }

        if (stickyGamepad1.right_bumper) {
            if (dumpBed.isDumping()) {
                dumpBed.retract();
            } else {
                dumpBed.dump();
            }
        }

        if (stickyGamepad1.dpad_up) {
            dumpBed.liftUp();
        }

        if (stickyGamepad1.dpad_down) {
            dumpBed.liftDown();
        }

//        if (stickyGamepad2.b) {
//            intake.grip();
//        }
//
//        if (stickyGamepad2.x) {
//            intake.release();
//        }
//
//        if (stickyGamepad2.y) {
//            intake.rotateUp();
//        }
//
//        if (stickyGamepad2.a) {
//            intake.rotateDown();
//        }

        drive.update();
        dumpBed.update();
//        intake.update();
    }
}

