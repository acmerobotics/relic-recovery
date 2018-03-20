package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SplineTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        Trajectory trajectory3 = new TrajectoryBuilder(new Pose2d(60, 0, Math.PI))
                .splineThrough(new Pose2d(24, -24, Math.PI), new Pose2d(12, -52, Math.PI / 2))
                .build();
        robot.drive.followTrajectory(trajectory3);
        robot.drive.waitForTrajectoryFollower();
    }
}
