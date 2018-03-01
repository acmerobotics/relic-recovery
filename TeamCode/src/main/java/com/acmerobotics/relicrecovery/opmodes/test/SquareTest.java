package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class SquareTest extends LinearOpMode {
    public static final Path SQUARE = new PathBuilder(new Pose2d(-24, -24, 0))
            .lineTo(new Vector2d(24, -24))
            .turn(Math.PI / 2)
            .lineTo(new Vector2d(24, 24))
            .turn(Math.PI / 2)
            .lineTo(new Vector2d(-24, 24))
            .turn(Math.PI / 2)
            .lineTo(new Vector2d(-24, -24))
            .turn(Math.PI / 2)
            .build();

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);

        robot.drive.setEstimatedPose(new Pose2d(-24, -24, 0));
        robot.drive.enablePositionEstimation();

        robot.start();

        waitForStart();

        while (opModeIsActive()) {
            robot.drive.followPath(SQUARE);
            while (opModeIsActive() && robot.drive.isFollowingPath()) {
                sleep(10);
            }
        }
    }
}
