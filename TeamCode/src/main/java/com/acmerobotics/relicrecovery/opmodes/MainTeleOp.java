package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
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

    private StickyGamepad stickyGamepad1;
    private Telemetry allTelemetry;
    private RobotDashboard dashboard;

    private MecanumDrive drive;

    private boolean halfSpeed;

    public MainTeleOp() {
        super(TELEOP_LOOP_TIME);
    }

    @Override
    protected void setup() {
        stickyGamepad1 = new StickyGamepad(gamepad1);

        dashboard = RobotDashboard.getInstance();

        OpModeConfiguration configuration = new OpModeConfiguration(hardwareMap.appContext);

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());

        drive = new MecanumDrive(hardwareMap, scheduler, subsystemTelemetry);
        drive.setEstimatedPose(initialPose);

        drive.registerLoops(looper);
    }

    @Override
    public void onLoop(double timestamp, double dt) {
        stickyGamepad1.update();

        if (stickyGamepad1.b) {
            halfSpeed = !halfSpeed;
        }

        double x, y = 0, omega;

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

        if (drive.getMode() == MecanumDrive.Mode.OPEN_LOOP || drive.getMode() == MecanumDrive.Mode.OPEN_LOOP_RAMP) {
            drive.setVelocity(new Vector2d(x, y), omega);
        } else if (x != 0 && y != 0 && omega != 0){
            drive.setVelocity(new Vector2d(x, y), omega);
        }
    }

    @Override
    protected void postLoop() {
        allTelemetry.update();
        dashboard.drawOverlay();
    }
}

