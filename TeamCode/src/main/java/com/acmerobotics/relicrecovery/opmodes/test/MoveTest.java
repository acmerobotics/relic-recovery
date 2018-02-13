package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.PathBuilder;

public class MoveTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followPath(new PathBuilder(robot.drive.getEstimatedPose())
                .forward(48)
                .build());
        robot.drive.waitForPathFollower();

        telemetry.addData("estimatedError", robot.drive.getEstimatedPose());
        telemetry.update();

        while (opModeIsActive());
    }
}
