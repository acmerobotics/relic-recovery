package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.mech.GlyphLift;
import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import static java.lang.Thread.sleep;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private MecanumDrive drive;
    private boolean fieldCentric, halfSpeed;
    private StickyGamepad stickyGamepad1;
    private Looper looper;
    private GlyphLift frontLift;


    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());
        stickyGamepad1 = new StickyGamepad(gamepad1);

        frontLift = new GlyphLift(hardwareMap, telemetry, GlyphLift.Side.FRONT);
        drive = new MecanumDrive(hardwareMap, telemetry, new Pose2d(0, 0, 0));

        looper = new Looper(20);
        looper.addLoop((timestamp, dt) -> {
            telemetry.update();
        });

        frontLift.registerLoops(looper);
        drive.registerLoops(looper);
        looper.start();

        frontLift.zeroLift();
    }

    @Override
    public void loop() {
        try {
            sleep(20);
        } catch (InterruptedException e) {
            e.printStackTrace();
        }

        stickyGamepad1.update();

        if (stickyGamepad1.a) {
            fieldCentric = !fieldCentric;
        }

        if (stickyGamepad1.y) {
            halfSpeed = !halfSpeed;
        }

        double leadScrewPower, pinionPower;

        if (gamepad1.y) {
            pinionPower = 1;
        } else if (gamepad1.a) {
            pinionPower = -1;
        } else {
            pinionPower = 0;
        }

        if (gamepad1.dpad_up) {
            leadScrewPower = 1;
        } else if (gamepad1.dpad_down) {
            leadScrewPower = -1;
        } else {
            leadScrewPower = 0;
        }

        frontLift.setLiftPower(leadScrewPower, pinionPower);

        if (gamepad1.left_bumper) {
            frontLift.setIntakePower(1, -0.5);
        } else if (gamepad1.right_bumper) {
            frontLift.setIntakePower(-1, 1);
        } else {
            frontLift.setIntakePower(0, 0);
        }

        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double omega = gamepad1.right_stick_x / 4;
        double heading = drive.getHeading();

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }
    }

    @Override
    public void stop() {
        looper.terminate();
    }

    @Override
    public void internalPostInitLoop() {

    }

    @Override
    public void internalPostLoop() {

    }
}

