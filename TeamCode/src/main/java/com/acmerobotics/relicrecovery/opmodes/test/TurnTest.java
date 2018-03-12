package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class TurnTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followTrajectory(new TrajectoryBuilder(robot.drive.getEstimatedPose())
                .turn(Math.PI / 2)
                .build());
        robot.drive.waitForTrajectoryFollower();

        telemetry.addData("heading", robot.drive.getHeading());
        telemetry.update();

        while (opModeIsActive());
    }
}
