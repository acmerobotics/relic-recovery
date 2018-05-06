package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.path.TrajectoryBuilder;
import com.acmerobotics.relicrecovery.subsystems.Robot;
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
                .lineToPose(new Pose2d(72, 0, Math.PI))
                .build());
        robot.drive.waitForTrajectoryFollower();
    }
}
