package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.path.SplineInterpolator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

@Autonomous
public class LinearPathTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        robot.drive.enablePositionEstimation();
        robot.start();

        waitForStart();

        robot.drive.followTrajectory(
                robot.drive.trajectoryBuilder(new Pose2d(0, 0, 0))
                .lineTo(new Vector2d(72, 0), new SplineInterpolator(0, Math.PI))
                .build());
        robot.drive.waitForTrajectoryFollower();
    }
}
