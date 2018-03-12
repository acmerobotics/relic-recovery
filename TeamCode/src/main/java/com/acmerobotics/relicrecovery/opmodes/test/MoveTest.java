package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class MoveTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getEstimatedPose())
                .forward(48)
                .build());
        robot.drive.waitForTrajectoryFollower();

        telemetry.addData("estimatedPose", robot.drive.getEstimatedPose());
        telemetry.update();

        while (opModeIsActive());
    }
}
