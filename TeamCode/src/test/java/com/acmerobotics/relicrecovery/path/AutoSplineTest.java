package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;

import org.junit.Ignore;
import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.drawTrajectoryOnField;

public class AutoSplineTest {
    @Ignore
    @Test
    public void testFourGlyphNear() {
        Trajectory trajectory = new TrajectoryBuilder(
                new Pose2d(BalancingStone.NEAR_BLUE.getPosition(), Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineThrough(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
                .beginComposite()
                .splineThrough(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
                .closeComposite()
                .splineThrough(new Pose2d(24, -12, Math.PI / 4))
                .beginComposite()
                .splineThrough(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60))
                .closeComposite()
                .build();
        drawTrajectoryOnField(trajectory, "fourGlyphNear");
        System.out.format("4 Glyph Near Duration: %.2fs\n", trajectory.duration());
    }
}
