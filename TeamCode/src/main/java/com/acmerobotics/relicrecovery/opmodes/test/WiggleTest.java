package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.path.SplineInterpolator;
import com.acmerobotics.splinelib.path.WiggleInterpolator;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class WiggleTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.intake.autoIntake();

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        Trajectory trajectory = AutoPaths.trajectoryBuilder(stonePose)
                .beginComposite()
                .lineTo(new Vector2d(24, stonePose.y()))
                .splineTo(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -24, 7 * Math.PI / 12))
                .splineTo(new Pose2d(-4, -10, 3 * Math.PI / 4),
                        new WiggleInterpolator(Math.toRadians(10), 6,
                                new SplineInterpolator(7 * Math.PI / 12, 3 * Math.PI / 4)))
                .closeComposite()
                .build();

        robot.drive.setEstimatedPose(stonePose);
        robot.drive.followTrajectory(trajectory);
        robot.drive.waitForTrajectoryFollower();
    }
}
