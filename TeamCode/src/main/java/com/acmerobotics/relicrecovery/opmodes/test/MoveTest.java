package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
public class MoveTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getEstimatedPose())
                .forward(48)
                .build());
        robot.drive.waitForTrajectoryFollower();

        telemetry.addData("estimatedPose", robot.drive.getEstimatedPose());
        telemetry.update();

        while (opModeIsActive());
    }
}
