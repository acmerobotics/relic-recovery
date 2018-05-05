package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.path.Trajectory;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
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
