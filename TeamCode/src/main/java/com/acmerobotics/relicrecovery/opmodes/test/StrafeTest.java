package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
@Config
public class StrafeTest extends AutoOpMode {
    public static double DISTANCE = AutoPaths.CRYPTO_COL_WIDTH;

    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.followPath(new PathBuilder(robot.drive.getEstimatedPose())
                .strafeLeft(DISTANCE)
                .build());
        robot.drive.waitForPathFollower();

        telemetry.addData("estimatedPose", robot.drive.getEstimatedPose());
        telemetry.update();

        while (opModeIsActive());
    }
}
