package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.configuration.OpModeConfiguration;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.mech.GlyphLift;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private MecanumDrive drive;
    private boolean halfSpeed;
    private StickyGamepad stickyGamepad1;
    private Looper looper;
    private GlyphLift frontLift;


    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);

        OpModeConfiguration configuration = new OpModeConfiguration(hardwareMap.appContext);

        RobotDashboard dashboard = RobotDashboard.getInstance();

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        Telemetry allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        frontLift = new GlyphLift(hardwareMap, subsystemTelemetry, GlyphLift.Side.FRONT);
        drive = new MecanumDrive(hardwareMap, subsystemTelemetry, new Pose2d(0, 0, 0));

        looper = new Looper(20);
        frontLift.registerLoops(looper);
        drive.registerLoops(looper);
        looper.addLoop((timestamp, dt) -> allTelemetry.update());
        looper.start();

        frontLift.zeroLift();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();

        if (stickyGamepad1.b) {
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

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        drive.setVelocity(new Vector2d(x, y), omega);
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

