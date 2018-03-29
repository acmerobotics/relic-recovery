package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
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
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineThrough(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
                .splineThrough(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
                .build();
        robot.drive.setEstimatedPose(trajectory.start());
        robot.drive.followTrajectory(trajectory);
        robot.drive.waitForTrajectoryFollower();
    }
}
