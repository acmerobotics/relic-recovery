package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.LineSegment;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * Created by ryanbrott on 12/4/17.
 */

@Autonomous(name = "MultiGlyph Auto")
public class MultiGlyphAuto extends LinearOpMode {
    private Path stoneToPit = new Path(Arrays.asList(
            new LineSegment(new Pose2d(48, -48, Math.PI), new Pose2d(12, -48, Math.PI)),
            new PointTurn(new Pose2d(12, -48, Math.PI), -Math.PI / 2),
            new LineSegment(new Pose2d(12, -48, Math.PI / 2), new Pose2d(12, -12, Math.PI / 2))
    ));

    private Path pitToCrypto = new Path(Arrays.asList(
            new LineSegment(new Pose2d(12, -12, Math.PI / 2), new Pose2d(12, -60, Math.PI / 2))
    ));

    private Path cryptoToPit = new Path(Arrays.asList(
            new LineSegment(new Pose2d(12, -60, Math.PI / 2), new Pose2d(12, -12, Math.PI / 2))
    ));

    private RobotDashboard dashboard;
    private Looper looper;
    private MecanumDrive drive;

    @Override
    public void runOpMode() throws InterruptedException {

        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry(), stoneToPit.getPose(0));

        looper = new Looper();
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            telemetry.update();
        }));
        looper.start();

        waitForStart();

        drive.followPath(stoneToPit);
        waitForPathFollower();

        for (int i = 0; i < 3; i++) {
            sleep(1500);
            drive.followPath(pitToCrypto);
            waitForPathFollower();

            sleep(1500);
            drive.followPath(cryptoToPit);
            waitForPathFollower();
        }
    }

    private void waitForPathFollower() {
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }
    }
}
