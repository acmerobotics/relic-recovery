package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.configuration.OpModeConfiguration;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.mech.GlyphLift;
import com.acmerobotics.relicrecovery.mech.Periscope;
import com.acmerobotics.relicrecovery.mech.RelicRecoverer;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.Arrays;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    private Looper looper;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private MecanumDrive drive;
    private GlyphLift frontLift;
    private Periscope periscope;
    private RelicRecoverer relicRecoverer;

    private boolean halfSpeed, secondControllerGlyph = true, changingMode;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        OpModeConfiguration configuration = new OpModeConfiguration(hardwareMap.appContext);

        RobotDashboard dashboard = RobotDashboard.getInstance();

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        Telemetry allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new MecanumDrive(hardwareMap, subsystemTelemetry, new Pose2d(0, 0, 0));
        frontLift = new GlyphLift(hardwareMap, subsystemTelemetry, GlyphLift.Side.FRONT);
        periscope = new Periscope(hardwareMap, subsystemTelemetry);
        relicRecoverer = new RelicRecoverer(hardwareMap, subsystemTelemetry);

        looper = new Looper(20);
        frontLift.registerLoops(looper);
        drive.registerLoops(looper);
        periscope.registerLoops(looper);
        looper.addLoop(relicRecoverer); // TODO: fix this?
        looper.addLoop((timestamp, dt) -> allTelemetry.update());
        looper.start();

//        frontLift.zeroLift();
//
//        periscope.raise();
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            if (!changingMode) {
                secondControllerGlyph = !secondControllerGlyph;
                changingMode = true;
            }
        } else {
            changingMode = false;
        }

        if (gamepad2.left_bumper && gamepad2.right_bumper) {
            secondControllerGlyph = !secondControllerGlyph;
        }

        if (stickyGamepad1.b) {
            halfSpeed = !halfSpeed;
        }

        double x, y = 0, omega;

        if (secondControllerGlyph) {
            y = (gamepad2.left_trigger - gamepad2.right_trigger) / 4;

            if (gamepad2.dpad_up) {
                frontLift.setLiftPower(1, 0);
            } else if (gamepad2.dpad_down) {
                frontLift.setLiftPower(-1, 0);
            } else if (gamepad2.dpad_left) {
                frontLift.setLiftPower(0, 1);
            } else if (gamepad2.dpad_right) {
                frontLift.setLiftPower(0, -1);
            } else if (frontLift.getLiftMode() == GlyphLift.LiftMode.OPEN_LOOP) {
                frontLift.setLiftPower(0, 0);
            }

//            if (stickyGamepad2.y) {
//                frontLift.setHeight(0.5);
//            } else if (stickyGamepad2.x) {
//                frontLift.setHeight(6.5);
//            } else if (stickyGamepad2.a) {
//                frontLift.setHeight(12.5);
//            } else if (stickyGamepad2.b) {
//                frontLift.setHeight(18.5);
//            }

            if (gamepad2.left_bumper) {
                frontLift.setIntakePower(-1, -1);
            } else if (gamepad2.right_bumper) {
                frontLift.intakeGlyph();
            }

            if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                frontLift.setIntakePower(-gamepad2.left_stick_y, -gamepad2.right_stick_y);
            }
        } else {
            if (gamepad2.dpad_up) {
                relicRecoverer.setPosition(RelicRecoverer.Position.UP);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                relicRecoverer.setPosition(RelicRecoverer.Position.CLOSED);
            } else if (gamepad2.dpad_down) {
                relicRecoverer.setPosition(RelicRecoverer.Position.OPEN);
            }
        }

        if (gamepad1.left_bumper) {
            frontLift.setIntakePower(-1, -1);
        } else if (gamepad1.right_bumper) {
            frontLift.intakeGlyph();
        }

        x = gamepad1.left_stick_y;

        if (gamepad1.left_stick_x != 0) {
            y = -gamepad1.left_stick_x;
        }

        omega = gamepad1.right_stick_x / 4;

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        double targetHeading = Double.NaN;
        if (gamepad1.dpad_left) {
            targetHeading = Math.PI / 2;
        } else if (gamepad1.dpad_down) {
            targetHeading = Math.PI;
        } else if (gamepad1.dpad_right) {
            targetHeading = -Math.PI / 2;
        } else if (gamepad1.dpad_up) {
            targetHeading = 0;
        }

        if (!Double.isNaN(targetHeading)) {
            Pose2d robotPose = drive.getEstimatedPose();
            double turnAngle = Angle.norm(targetHeading - robotPose.heading());
            Path turn = new Path(Arrays.asList(
                    new PointTurn(robotPose, turnAngle)
            ));
            drive.followPath(turn);
        }

        if (drive.getMode() == MecanumDrive.Mode.OPEN_LOOP || drive.getMode() == MecanumDrive.Mode.OPEN_LOOP_RAMP) {
            drive.setVelocity(new Vector2d(x, y), omega, true);
        } else if (x != 0 && y != 0 && omega != 0){
            drive.setVelocity(new Vector2d(x, y), omega, true);
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

