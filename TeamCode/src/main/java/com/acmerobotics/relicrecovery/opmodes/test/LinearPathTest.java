package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
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
                new TrajectoryBuilder(new Pose2d(0, 0, 0))
                .lineToPose(new Pose2d(72, 0, Math.PI))
                .build());
        robot.drive.waitForTrajectoryFollower();
    }
}
