package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.Dump;
import com.acmerobotics.relicrecovery.subsystems.Intake;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.PhoneSwivel;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp(name = "TeleOp", group = "teleop")
public class MainTeleOp extends ScheduledLoopOpMode {
    public static final double TELEOP_LOOP_TIME = 0.02;

    public static Pose2d initialPose = new Pose2d(0, 0, 0);

    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private Telemetry allTelemetry;
    private RobotDashboard dashboard;

    private MecanumDrive drive;
    private Dump dump;
    private JewelSlapper jewelSlapper;
    private Intake intake;
    private PhoneSwivel swivel;

    private boolean halfSpeed;

    public MainTeleOp() {
        super(TELEOP_LOOP_TIME);
    }

    @Override
    protected void setup() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        dashboard = RobotDashboard.getInstance();

        OpModeConfiguration configuration = new OpModeConfiguration(hardwareMap.appContext);

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());

        drive = new MecanumDrive(hardwareMap, scheduler, subsystemTelemetry);
        drive.setEstimatedPose(initialPose);

        dump = new Dump(hardwareMap, scheduler);

        jewelSlapper = new JewelSlapper(hardwareMap, scheduler);

        intake = new Intake(hardwareMap, scheduler);

        swivel = new PhoneSwivel(hardwareMap, scheduler);

        drive.registerLoops(looper);
        dump.registerLoops(looper);
        intake.registerLoops(looper);
    }

    @Override
    public void onLoop(double timestamp, double dt) {
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
            if (dump.isDown()) {
                dump.rotateUp();
            } else {
                dump.rotateDown();
            }
        }

        if (stickyGamepad1.left_bumper) {
            if (dump.isReleaseEngaged()) {
                dump.disengageRelease();
            } else {
                dump.engageRelease();
            }
        }

        if (gamepad1.left_trigger > 0.8) {
            swivel.pointAtJewel();
        }

        if (gamepad1.right_trigger > 0.8) {
            swivel.pointAtCryptobox();
        }

        if (stickyGamepad1.dpad_up) {
            jewelSlapper.undeploy();
        }

        if (stickyGamepad1.dpad_down) {
            jewelSlapper.deploy();
        }

        if (stickyGamepad1.dpad_left) {
            jewelSlapper.setPosition(JewelSlapper.Position.LEFT);
        }

        if (stickyGamepad1.dpad_right) {
            jewelSlapper.setPosition(JewelSlapper.Position.RIGHT);
        }

        if (stickyGamepad1.x) {
            jewelSlapper.setPosition(JewelSlapper.Position.CENTER);
        }

        if (stickyGamepad2.b) {
            intake.grip();
        }

        if (stickyGamepad2.x) {
            intake.release();
        }

        if (stickyGamepad2.y) {
            intake.rotateUp();
        }

        if (stickyGamepad2.a) {
            intake.rotateDown();
        }

    }

    @Override
    protected void postLoop() {
        allTelemetry.update();
        dashboard.drawOverlay();
    }
}

