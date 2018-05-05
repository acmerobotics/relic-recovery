package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.path.ParametricSegment;
import com.acmerobotics.library.path.TrajectorySegment;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.library.path.parametric.CompositePath;
import com.acmerobotics.library.path.parametric.ParametricPath;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

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

        TrajectorySegment motionSegment = new ParametricSegment(spline, MecanumDrive.AXIAL_CONSTRAINTS);
        writePathMotionSegment(motionSegment, "simpleMotionSegment");
    }
}
