package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.parametric.CompositePath;
import com.acmerobotics.relicrecovery.path.parametric.ParametricPath;

import org.junit.Ignore;
import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.writeParametricPath;
import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.writePathMotionSegment;

public class SplinePathTest {
    @Ignore
    @Test
    public void testSplineGeneration() {
        ParametricPath spline = CompositePath.fitSpline(
                new Pose2d(0, -12, 3 * Math.PI / 4),
                new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2));
        writeParametricPath(spline, "simpleSpline");

        TrajectorySegment motionSegment = new ParametricSegment(spline);
        writePathMotionSegment(motionSegment, "simpleMotionSegment");
    }
}
