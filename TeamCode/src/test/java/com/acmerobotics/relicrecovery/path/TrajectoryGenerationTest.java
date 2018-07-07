package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.vision.OpenCVStaticLoader;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.trajectory.Trajectory;

import org.junit.Ignore;
import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.drawTrajectoryOnField;
import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.writeTrajectory;

public class TrajectoryGenerationTest {
    @Ignore
    @Test
    public void drawTrajectory() {
        OpenCVStaticLoader.loadStaticLibs();
        Trajectory trajectory = AutoPaths.trajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineTo(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
                .splineTo(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60, Math.PI / 2))
                .splineTo(new Pose2d(24, -12, Math.PI / 4))
                .splineTo(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60, Math.PI / 2))
                .splineTo(new Pose2d(16, 0, 3 * Math.PI / 8))
                .splineTo(new Pose2d(12, -60, Math.PI / 2))
                .build();
        Trajectory trajectory2 = AutoPaths.trajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineTo(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
//                .beginComposite()
                .splineTo(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
//                .closeComposite()
                .splineTo(new Pose2d(24, -12, Math.PI / 4))
//                .beginComposite()
                .splineTo(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60))
//                .closeComposite()
                .splineTo(new Pose2d(16, 0, 3 * Math.PI / 8))
//                .beginComposite()
                .splineTo(new Pose2d(12, -44, Math.PI / 2))
                .lineTo(new Vector2d(12, -60))
//                .closeComposite()
                .build();
        System.out.println("Trajectory 1 duration: " + trajectory.duration());
        System.out.println("Trajectory 2 duration: " + trajectory2.duration());
        drawTrajectoryOnField(trajectory, "trajectory");
        drawTrajectoryOnField(trajectory2, "trajectory2");
        writeTrajectory(trajectory, "trajectory");
        writeTrajectory(trajectory2, "trajectory2");
    }
}
