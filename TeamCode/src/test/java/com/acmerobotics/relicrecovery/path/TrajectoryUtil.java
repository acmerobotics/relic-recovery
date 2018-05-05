package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.path.ParametricSegment;
import com.acmerobotics.library.path.Trajectory;
import com.acmerobotics.library.path.TrajectorySegment;
import com.acmerobotics.library.path.parametric.CompositePath;
import com.acmerobotics.library.path.parametric.LinePath;
import com.acmerobotics.library.path.parametric.ParametricPath;
import com.acmerobotics.library.path.parametric.SplinePath;
import com.acmerobotics.relicrecovery.vision.MatOverlay;
import com.acmerobotics.relicrecovery.vision.OpenCVStaticLoader;
import com.acmerobotics.relicrecovery.vision.Overlay;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class TrajectoryUtil {
    public static void writeParametricPath(ParametricPath path, String name) {
        try {
            File outputDir = new File("../out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
            System.out.printf("writing path: length=%.2fin", path.length());
            for (int i = 0; i <= 200; i++) {
                double displacement = i / 200.0 * path.length();
                Pose2d pose = path.getPose(displacement);
                Pose2d velocity = path.getDerivative(displacement);
                Pose2d acceleration = path.getSecondDerivative(displacement);
                writer.write(String.format("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", displacement,
                        pose.x(), pose.y(), pose.heading(),
                        velocity.x(), velocity.y(), velocity.heading(),
                        acceleration.x(), acceleration.y(), acceleration.heading()));
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writePathMotionSegment(TrajectorySegment motionSegment, String name) {
        try {
            File outputDir = new File("../out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
            for (int i = 0; i <= 200; i++) {
                double time = i / 200.0 * motionSegment.duration();
                Pose2d pose = motionSegment.getPose(time);
                Pose2d velocity = motionSegment.getVelocity(time);
                Pose2d acceleration = motionSegment.getAcceleration(time);
                writer.write(String.format("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", time,
                        pose.x(), pose.y(), pose.heading(),
                        velocity.x(), velocity.y(), velocity.heading(),
                        acceleration.x(), acceleration.y(), acceleration.heading()));

            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writeTrajectory(Trajectory trajectory, String name) {
        try {
            File outputDir = new File("../out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
            System.out.println("starting duration: " + trajectory.duration());
            for (double time = 0; time < trajectory.duration(); time += 0.01) {
                Pose2d pose = trajectory.getPose(time);
                Pose2d velocity = trajectory.getVelocity(time);
                Pose2d acceleration = trajectory.getAcceleration(time);
                writer.write(String.format("%f, %f, %f, %f, %f, %f, %f, %f, %f, %f\n", time,
                        pose.x(), pose.y(), pose.heading(),
                        velocity.x(), velocity.y(), velocity.heading(),
                        acceleration.x(), acceleration.y(), acceleration.heading()));

            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void drawTrajectoryOnField(Trajectory trajectory, String name) {
        OpenCVStaticLoader.loadStaticLibs();
        Mat field = Imgcodecs.imread("../dashboard/src/assets/field.png");
        MatOverlay overlay = new MatOverlay(field);
        overlay.setScalingFactor(field.width() / 144.0);
        drawTrajectory(overlay, trajectory, new Scalar(0, 255, 0));
        for (double t = 0; t <= trajectory.duration(); t += 0.25) {
            Vector2d pos = trajectory.getPose(t).pos();
            overlay.fillCircle(new Point(-pos.y() + 72, -pos.x() + 72), 0.5, new Scalar(0, 255, 255));
        }
        File outputDir = new File("../out");
        outputDir.mkdirs();
        Imgcodecs.imwrite(new File(outputDir, name + ".png").getAbsolutePath(), field);
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
