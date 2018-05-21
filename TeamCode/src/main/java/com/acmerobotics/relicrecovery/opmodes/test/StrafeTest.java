package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous
@Config
public class StrafeTest extends AutoOpMode {
    public static double DISTANCE = AutoPaths.CRYPTO_COL_WIDTH;

    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(robot.drive.getEstimatedPose())
                .strafeLeft(DISTANCE)
                .build());
        robot.drive.waitForTrajectoryFollower();

        telemetry.addData("estimatedPose", robot.drive.getEstimatedPose());
        telemetry.update();

        while (opModeIsActive());
    }
}
