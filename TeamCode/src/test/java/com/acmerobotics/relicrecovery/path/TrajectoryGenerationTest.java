package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.vision.OpenCVStaticLoader;

import org.junit.Ignore;
import org.junit.Test;

import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.drawTrajectoryOnField;
import static com.acmerobotics.relicrecovery.path.TrajectoryUtil.writeTrajectory;

public class TrajectoryGenerationTest {
    @Ignore
    @Test
    public void drawTrajectory() {
        OpenCVStaticLoader.loadStaticLibs();
        System.load("R:\\Downloads\\opencv\\build\\java\\x64\\opencv_java331.dll");
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineThrough(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
                .splineThrough(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60, Math.PI / 2))
                .splineThrough(new Pose2d(24, -12, Math.PI / 4))
                .splineThrough(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60, Math.PI / 2))
                .splineThrough(new Pose2d(16, 0, 3 * Math.PI / 8))
                .splineThrough(new Pose2d(12, -60, Math.PI / 2))
                .build();
        Trajectory trajectory2 = new TrajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineThrough(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
//                .beginComposite()
                .splineThrough(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
//                .closeComposite()
                .splineThrough(new Pose2d(24, -12, Math.PI / 4))
//                .beginComposite()
                .splineThrough(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60))
//                .closeComposite()
                .splineThrough(new Pose2d(16, 0, 3 * Math.PI / 8))
//                .beginComposite()
                .splineThrough(new Pose2d(12, -44, Math.PI / 2))
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
