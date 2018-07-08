package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.path.Path;
import com.acmerobotics.splinelib.path.QuinticSplineSegment;
import com.acmerobotics.splinelib.path.TangentInterpolator;
import com.acmerobotics.splinelib.path.WiggleInterpolator;
import com.acmerobotics.splinelib.trajectory.PathTrajectorySegment;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.acmerobotics.splinelib.trajectory.TrajectorySegment;

import org.junit.Ignore;
import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.drawTrajectoryOnField;

public class AutoSplineTest {
//    @Ignore
    @Test
    public void testFiveGlyphNear() {
        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        Trajectory trajectory = AutoPaths.trajectoryBuilder(stonePose)
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, stonePose.y()))
                .turnTo(Math.PI / 2)
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -56))
                .waitFor(0.5)
                .beginComposite()
                .splineTo(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -24, 7 * Math.PI / 12))
                .splineTo(new Pose2d(-4, -10, 3 * Math.PI / 4),
                        new WiggleInterpolator(Math.toRadians(15), 6, new TangentInterpolator()))
                .closeComposite()
                .build();
        for (TrajectorySegment segment : trajectory.getSegments()) {
            System.out.println(segment.duration());
        }
        Path splineSegment = ((PathTrajectorySegment) trajectory.getSegments().get(4)).getPaths().get(0);
        drawTrajectoryOnField(trajectory, "fiveGlyphNear");
        System.out.format("5 Glyph Near Duration: %.2fs\n", trajectory.duration());
    }
}
