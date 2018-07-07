package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class SplineTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        Trajectory trajectory = robot.drive.trajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineTo(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
                .splineTo(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
                .build();
        robot.drive.setEstimatedPose(trajectory.start());
        robot.drive.followTrajectory(trajectory);
        robot.drive.waitForTrajectoryFollower();
    }
}
