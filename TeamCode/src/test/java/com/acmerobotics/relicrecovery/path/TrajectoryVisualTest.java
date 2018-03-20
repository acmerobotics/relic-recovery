package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.parametric.CompositePath;
import com.acmerobotics.relicrecovery.path.parametric.LinePath;
import com.acmerobotics.relicrecovery.path.parametric.ParametricPath;
import com.acmerobotics.relicrecovery.path.parametric.SplinePath;
import com.acmerobotics.relicrecovery.vision.MatOverlay;
import com.acmerobotics.relicrecovery.vision.Overlay;

import org.junit.Ignore;
import org.junit.Test;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;

public class TrajectoryVisualTest {
    @Ignore
    @Test
    public void drawTrajectory() {
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
                .beginComposite()
                .splineThrough(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
                .closeComposite()
                .splineThrough(new Pose2d(24, -12, Math.PI / 4))
                .beginComposite()
                .splineThrough(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60))
                .closeComposite()
                .splineThrough(new Pose2d(16, 0, 3 * Math.PI / 8))
                .beginComposite()
                .splineThrough(new Pose2d(12, -44, Math.PI / 2))
                .lineTo(new Vector2d(12, -60))
                .closeComposite()
                .build();
        System.out.println("Trajectory 1 duration: " + trajectory.duration());
        System.out.println("Trajectory 2 duration: " + trajectory2.duration());
        writeTrajectory(trajectory, "trajectory");
        writeTrajectory(trajectory2, "trajectory2");
    }

    public static void writeTrajectory(Trajectory trajectory, String name) {
        Mat field = Imgcodecs.imread("dashboard/src/assets/field.png");
        MatOverlay overlay = new MatOverlay(field);
        overlay.setScalingFactor(field.width() / 144.0);
        drawTrajectory(overlay, trajectory, new Scalar(0, 255, 0));
        Imgcodecs.imwrite(name + ".png", field);
    }

    public static void drawTrajectory(Overlay overlay, Trajectory trajectory, Scalar color) {
        for (TrajectorySegment motionSegment : trajectory.segments()) {
            if (motionSegment instanceof ParametricSegment) {
                drawParametricPath(overlay, ((ParametricSegment) motionSegment).path(), color);
            }
        }
    }

    public static void drawParametricPath(Overlay overlay, ParametricPath path, Scalar color) {
        if (path instanceof CompositePath) {
            for (ParametricPath subPath : ((CompositePath) path).segments()) {
                drawParametricPath(overlay, subPath, color);
            }
        } else if (path instanceof LinePath) {
            LinePath line = (LinePath) path;
            overlay.strokeLine(new Point(-line.start().y() + 72, -line.start().x() + 72), new Point(-line.end().y() + 72, -line.end().x() + 72), color, 8);
        } else if (path instanceof SplinePath) {
            SplinePath spline = (SplinePath) path;
            Point lastPoint = new Point(-spline.yOffset() + 72, -spline.xOffset() + 72);
            for (int i = 1; i <= 100; i++) {
                double t = i / 100.0 * spline.length();
                Pose2d pose = spline.getPose(t);
                Point point = new Point(-pose.y() + 72, -pose.x() + 72);
                overlay.strokeLine(lastPoint, point, color, 8);
                lastPoint = point;
            }
        }
    }
}
