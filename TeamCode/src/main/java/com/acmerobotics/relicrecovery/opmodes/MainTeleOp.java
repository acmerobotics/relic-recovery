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
import com.acmerobotics.relicrecovery.mech.Periscope;
import com.acmerobotics.relicrecovery.mech.RelicRecoverer;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends OpMode {
    public static Pose2d initialPose = new Pose2d(0, 0, 0);

    private Looper looper;
    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private MecanumDrive drive;
    private GlyphLift frontLift;
    private Periscope periscope;
    private RelicRecoverer relicRecoverer;

    private boolean halfSpeed, secondControllerGlyph = true, changingMode;

    private VisionCamera camera;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        OpModeConfiguration configuration = new OpModeConfiguration(hardwareMap.appContext);

        RobotDashboard dashboard = RobotDashboard.getInstance();

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        Telemetry allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());

        camera = new VisionCamera();
        camera.setImageDir(LoggingUtil.getImageDir(this));
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        drive = new MecanumDrive(hardwareMap, subsystemTelemetry, initialPose);
        frontLift = new GlyphLift(hardwareMap, subsystemTelemetry, GlyphLift.Side.FRONT);
        periscope = new Periscope(hardwareMap, subsystemTelemetry);
        relicRecoverer = new RelicRecoverer(hardwareMap, subsystemTelemetry);

        looper = new Looper(20);
        frontLift.registerLoops(looper);
        drive.registerLoops(looper);
        periscope.registerLoops(looper);
        looper.addLoop(relicRecoverer); // TODO: fix this?
        looper.addLoop((timestamp, dt) -> {
            allTelemetry.update();
            dashboard.drawOverlay();
        });
        looper.start();

//        drive.setMaintainHeading(true);

        frontLift.zeroLift();

        periscope.raise();
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

        if (stickyGamepad1.b) {
            halfSpeed = !halfSpeed;
        }

        double x, y = 0, omega;

        if (secondControllerGlyph) {
            y = (gamepad2.left_trigger - gamepad2.right_trigger) / 4.0;

            double leadScrewPower = Double.NaN, pinionPower = Double.NaN;
            if (gamepad2.dpad_up) {
                leadScrewPower = 1;
            } else if (gamepad2.dpad_down) {
                leadScrewPower = -1;
            } else if (frontLift.getLiftMode() == GlyphLift.LiftMode.OPEN_LOOP) {
                leadScrewPower = 0;
            }

            if (gamepad2.dpad_left) {
                pinionPower = 1;
            } else if (gamepad2.dpad_right) {
                pinionPower = -1;
            } else if (frontLift.getLiftMode() == GlyphLift.LiftMode.OPEN_LOOP) {
                pinionPower = 0;
            }

            if (!Double.isNaN(leadScrewPower)) {
                frontLift.setLeadScrewPower(leadScrewPower);
            }

            if (!Double.isNaN(pinionPower)) {
                frontLift.setPinionPower(pinionPower);
            }

            if (stickyGamepad2.y) {
                frontLift.setHeight(0.5);
            } else if (stickyGamepad2.x) {
                frontLift.setHeight(6.5);
            } else if (stickyGamepad2.a) {
                frontLift.setHeight(12.5);
            } else if (stickyGamepad2.b) {
                frontLift.setHeight(18.5);
            }

            if (gamepad2.left_bumper) {
                frontLift.setIntakePower(-1, -1);
            } else if (gamepad2.right_bumper) {
//                frontLift.intakeGlyph();
                frontLift.setIntakePower(1, 1);
            } else if (frontLift.getIntakeMode() == GlyphLift.IntakeMode.OPEN_LOOP) {
                frontLift.setIntakePower(0, 0);
            }

            if (gamepad2.left_stick_y != 0 || gamepad2.right_stick_y != 0) {
                frontLift.setIntakePower(gamepad2.left_stick_y, gamepad2.right_stick_y);
            }
        } else {
            if (gamepad2.dpad_up) {
                relicRecoverer.setPosition(RelicRecoverer.Position.UP);
            } else if (gamepad2.dpad_left || gamepad2.dpad_right) {
                relicRecoverer.setPosition(RelicRecoverer.Position.CLOSED);
            } else if (gamepad2.dpad_down) {
                relicRecoverer.setPosition(RelicRecoverer.Position.OPEN);
            }

            relicRecoverer.setExtendSpeed(gamepad2.right_stick_y);
            relicRecoverer.setOffsetSpeed(gamepad2.left_stick_y);

            frontLift.setLiftPower(0, 0);
            frontLift.setIntakePower(0, 0);
        }

        x = 0.75 * -gamepad1.left_stick_y;

        if (Math.abs(gamepad1.left_stick_x) > 0.5) {
            y = -gamepad1.left_stick_x;
        }

        if (gamepad1.left_trigger != 0 || gamepad1.right_trigger != 0) {
            y = (gamepad1.left_trigger - gamepad1.right_trigger) / 4.0;
        }

        omega = -gamepad1.right_stick_x / 24.0;

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

//        double targetHeading = Double.NaN;
//        if (gamepad1.dpad_left) {
//            targetHeading = Math.PI / 2;
//        } else if (gamepad1.dpad_down) {
//            targetHeading = Math.PI;
//        } else if (gamepad1.dpad_right) {
//            targetHeading = -Math.PI / 2;
//        } else if (gamepad1.dpad_up) {
//            targetHeading = 0;
//        }
//
//        if (!Double.isNaN(targetHeading)) {
//            Pose2d robotPose = drive.getEstimatedPose();
//            double turnAngle = Angle.norm(targetHeading - robotPose.heading());
//            Path turn = new Path(Arrays.asList(
//                    new PointTurn(robotPose, turnAngle)
//            ));
//            drive.followPath(turn);
//        }

        if (drive.getMode() == MecanumDrive.Mode.OPEN_LOOP || drive.getMode() == MecanumDrive.Mode.OPEN_LOOP_RAMP) {
            drive.setVelocity(new Vector2d(x, y), omega);
        } else if (x != 0 && y != 0 && omega != 0){
            drive.setVelocity(new Vector2d(x, y), omega);
        }

        telemetry.addData("secondControllerMode", secondControllerGlyph ? "GLYPH" : "RELIC");
    }
}

