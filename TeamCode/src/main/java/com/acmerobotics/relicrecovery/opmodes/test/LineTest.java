package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ryanbrott on 11/7/17.
 */

@Autonomous(name = "Line Test", group = "test")
public class LineTest extends LinearOpMode {
    private RobotDashboard dashboard;
    private MecanumDrive drive;
    private Looper looper;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry(), new Pose2d(60, 0, Math.PI));

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop((timestamp, dt) -> {
            dashboard.drawOverlay();
            telemetry.update();
        });
        looper.start();

        waitForStart();

        while (opModeIsActive()) {
            drive.followPath(new PathBuilder(new Pose2d(60, 0, Math.PI)).lineTo(new Vector2d(-60, 0)).build());
            waitForPathFollower();

            sleep(1500);

            drive.followPath(new PathBuilder(new Pose2d(-60, 0, Math.PI)).turn(Math.PI).build());
            waitForPathFollower();

            sleep(1500);

            drive.followPath(new PathBuilder(new Pose2d(-60, 0, 0)).lineTo(new Vector2d(60, 0)).build());
            waitForPathFollower();

            sleep(1500);

            drive.followPath(new PathBuilder(new Pose2d(60, 0, 0)).turn(Math.PI).build());
            waitForPathFollower();

            sleep(1500);
        }
    }

    private void waitForPathFollower() {
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }
    }
}
