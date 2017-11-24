package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * Created by ryanbrott on 11/7/17.
 */

@Disabled
@Autonomous(name = "Line Test", group = "test")
public class LineTest extends LinearOpMode {
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;
    private MecanumDrive drive;
    private Looper looper;

    @Override
    public void runOpMode() throws InterruptedException {
        Path path = Path.createFromPoses(Arrays.asList(
                new Pose2d(0, 24),
                new Pose2d(72, 24)
        ));

        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry(), new Pose2d(0, 0, 0));

        looper = new Looper(20);
        drive.registerLoops(looper);
        looper.start();

        waitForStart();

        drive.followPath(path);

        while (opModeIsActive()) {
//            Pose2d estimatedPose = drive.getEstimatedPose();
//
//            fieldOverlay.setFill("green");
//            fieldOverlay.fillCircle(estimatedPose.x(), estimatedPose.y(), 5);
//            dashboard.drawOverlay();
//
//            telemetry.addData("x", estimatedPose.x());
//            telemetry.addData("Y", estimatedPose.y());
//            telemetry.update();

            sleep(20);
        }
    }
}
