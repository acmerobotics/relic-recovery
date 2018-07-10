package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

//@Disabled
@TeleOp
public class SquareTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        robot.drive.setEstimatedPose(new Pose2d(-24, -24, 0));
        robot.drive.enablePositionEstimation();

        robot.start();

        waitForStart();

        Trajectory square = robot.drive.trajectoryBuilder(new Pose2d(-24, -24, 0))
                .lineTo(new Vector2d(24, -24))
                .turn(Math.PI / 2)
                .lineTo(new Vector2d(24, 24))
                .turn(Math.PI / 2)
                .lineTo(new Vector2d(-24, 24))
                .turn(Math.PI / 2)
                .lineTo(new Vector2d(-24, -24))
                .turn(Math.PI / 2)
                .build();

        while (opModeIsActive()) {
            robot.drive.followTrajectory(square);
            while (opModeIsActive() && robot.drive.isFollowingTrajectory()) {
                sleep(10);
            }
        }
    }
}
