package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.path.Trajectory;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;

import org.junit.Ignore;
import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.drawTrajectoryOnField;

public class AutoSplineTest {
    @Ignore
    @Test
    public void testSixGlyphNear() {
        Trajectory trajectory = AutoPaths.trajectoryBuilder(
                new Pose2d(BalancingStone.NEAR_BLUE.getPosition(), Math.PI))
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -48))
                .turnTo(Math.PI / 2)
//                .turnTo(Math.PI / 2)
//                .splineThrough(new Pose2d(0, -12, 3 * Math.PI / 4))
////                .beginComposite()
//                .splineThrough(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
//                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
////                .closeComposite()
//                .splineThrough(new Pose2d(28, -16, Math.PI / 4))
////                .beginComposite()
//                .splineThrough(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
//                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60))
////                .closeComposite()
//                .splineThrough(new Pose2d(16, 0, 3 * Math.PI / 8))
////                .beginComposite()
//                .splineThrough(new Pose2d(12, -44, Math.PI / 2))
//                .lineTo(new Vector2d(12, -60))
//                .closeComposite()
                .build();
        drawTrajectoryOnField(trajectory, "sixGlyphNear");
        System.out.format("6 Glyph Near Duration: %.2fs\n", trajectory.duration());
    }

    @Ignore
    @Test
    public void testSixGlyphFar() {
        Trajectory trajectory = AutoPaths.trajectoryBuilder(
                new Pose2d(BalancingStone.FAR_BLUE.getPosition(), Math.PI))
                .turnTo(Math.PI / 2)
                .beginComposite()
                .lineTo(new Vector2d(-24, -36))
                .splineThrough(new Pose2d(-8, -8, Math.PI / 4))
                .closeComposite()
                .splineThrough(new Pose2d(-24, -12, 0), new Pose2d(-56, -36 - AutoPaths.CRYPTO_COL_WIDTH, 0))
                .splineThrough(new Pose2d(-24, -12, 0), new Pose2d(0, 0, Math.PI / 4))
                .splineThrough(new Pose2d(-16, -12, 0), new Pose2d(-40, -36 + AutoPaths.CRYPTO_COL_WIDTH, 0))
                .lineTo(new Vector2d(-56, -36 + AutoPaths.CRYPTO_COL_WIDTH))
// .splineThrough(new Pose2d(-24, -12, 0), new Pose2d(12, 0, 0))
//                .splineThrough(new Pose2d(-24, -12, 0), new Pose2d(-56, -36, 0))
                .build();
        drawTrajectoryOnField(trajectory, "sixGlyphFar");
        System.out.format("6 Glyph Far Duration: %.2fs\n", trajectory.duration());
    }
}
