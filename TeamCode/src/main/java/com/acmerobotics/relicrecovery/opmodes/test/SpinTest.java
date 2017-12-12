package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 12/8/17.
 */

@Config
@TeleOp(name = "Spin Test")
public class SpinTest extends LinearOpMode {
    public static double SPIN_POWER = 0.3;

    private RobotDashboard dashboard;
    private Looper looper;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry());

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> telemetry.update()));
        looper.start();

        waitForStart();

        while (opModeIsActive()) {
            drive.setVelocity(new Vector2d(0, 0), SPIN_POWER);
        }
    }
}
