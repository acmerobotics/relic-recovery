package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * @author Ryan
 */

@Autonomous
public class Auto extends LinearOpMode {
    private RobotDashboard dashboard;
    private Looper looper;

    private MecanumDrive drive;

    private VisionCamera camera;
    private DynamicJewelTracker jewelTracker;

    @Override
    public void runOpMode() throws InterruptedException {
        // we're blue
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        looper = new Looper(20);

        drive = new MecanumDrive(hardwareMap);
        drive.registerLoops(looper);

        camera = new VisionCamera(hardwareMap.appContext);
        jewelTracker = new DynamicJewelTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        waitForStart();

        looper.start();

        Pose2d initialPose = new Pose2d(48, -48, Math.PI);
        Pose2d jewelPose = new Pose2d(initialPose.pos(), jewelTracker.getLeftColor() == DynamicJewelTracker.JewelColor.BLUE ? 3 * Math.PI / 4 : 5 * Math.PI / 4);
        Path jewelTurn = Path.createFromPoses(Arrays.asList(
                initialPose,
                jewelPose
        ));

        drive.followPath(jewelTurn);
        while (opModeIsActive() && drive.isFollowingPath()) {
            telemetry.addData("x", drive.getEstimatedPose().x());
            telemetry.addData("y", drive.getEstimatedPose().y());
            telemetry.update();
            sleep(10);
        }

        Path pathToCryptobox = Path.createFromPoses(Arrays.asList(
                jewelPose,
                new Pose2d(12, -48),
                new Pose2d(12, -60, Math.PI / 2)
        ));

        drive.followPath(pathToCryptobox);
        while (opModeIsActive() && drive.isFollowingPath()) {
            telemetry.addData("x", drive.getEstimatedPose().x());
            telemetry.addData("y", drive.getEstimatedPose().y());
            telemetry.update();
            sleep(10);
        }

        while (opModeIsActive()) {
            telemetry.addData("x", drive.getEstimatedPose().x());
            telemetry.addData("y", drive.getEstimatedPose().y());
            telemetry.update();
            sleep(20);
        }

        looper.terminate();
        camera.close();
    }
}
