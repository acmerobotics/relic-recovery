package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.path.TangentInterpolator;
import com.acmerobotics.splinelib.path.WiggleInterpolator;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.acmerobotics.splinelib.trajectory.TrajectorySegment;

import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.drawTrajectoryOnField;
import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.writeTrajectory;

public class AutoPathTest {
//    @Ignore
    @Test
    public void testFiveGlyphNear() {
        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        Trajectory trajectory = AutoPaths.trajectoryBuilder(stonePose)
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, stonePose.y()))
                .turnTo(Math.PI / 2)
                .reverse()
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -56))
                // deposit
                .reverse()
                .beginComposite()
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44))
                .splineTo(new Pose2d(16, -24, Math.PI / 3))
                .splineTo(new Pose2d(24, -10, Math.PI / 4),
                        new WiggleInterpolator(Math.toRadians(15), 6, new TangentInterpolator()))
                .closeComposite()
                // collect
                .reverse()
                .beginComposite()
                .splineTo(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -24, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44))
                .closeComposite()
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -56))
                // deposit
                .reverse()
                .beginComposite()
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44))
                .splineTo(new Pose2d(8, -24, 2 * Math.PI / 3))
                .splineTo(new Pose2d(0, -10, 3 * Math.PI / 4),
                        new WiggleInterpolator(Math.toRadians(15), 6, new TangentInterpolator()))
                .closeComposite()
                // collect
                .reverse()
                .beginComposite()
                .splineTo(new Pose2d(12, -24, Math.PI / 2))
                .lineTo(new Vector2d(12, -44))
                .closeComposite()
                .lineTo(new Vector2d(12, -56))
                // deposit
                .build();
        for (TrajectorySegment segment : trajectory.getSegments()) {
            System.out.println(segment.duration());
        }
        drawTrajectoryOnField(trajectory, "fiveGlyphNear");
        writeTrajectory(trajectory, "fiveGlyphNear");
        System.out.format("5 Glyph Near Duration: %.2fs\n", trajectory.duration());
    }
}
