package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TurnTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followPath(new PathBuilder(robot.drive.getEstimatedPose())
                .turn(Math.PI / 2)
                .build());
        robot.drive.waitForPathFollower();

        telemetry.addData("heading", robot.drive.getHeading());
        telemetry.update();

        while (opModeIsActive());
    }
}
