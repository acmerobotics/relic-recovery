package com.acmerobotics.relicrecovery.path;
import com.acmerobotics.library.vision.MatOverlay;
import com.acmerobotics.relicrecovery.vision.OpenCVStaticLoader;
import com.acmerobotics.library.vision.Overlay;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.path.LineSegment;
import com.acmerobotics.splinelib.path.ParametricCurve;
import com.acmerobotics.splinelib.path.Path;
import com.acmerobotics.splinelib.path.QuinticSplineSegment;
import com.acmerobotics.splinelib.trajectory.PathTrajectorySegment;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.acmerobotics.splinelib.trajectory.TrajectorySegment;

import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class TrajectoryUtil {
    public static void writePath(Path path, String name) {
        try {
            File outputDir = new File("../out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
            System.out.printf("writing path: length=%.2fin", path.length());
            for (int i = 0; i <= 200; i++) {
                double displacement = i / 200.0 * path.length();
                Pose2d pose = path.get(displacement);
                Pose2d velocity = path.deriv(displacement);
                Pose2d acceleration = path.secondDeriv(displacement);
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

    public static void writeTrajectorySegment(TrajectorySegment trajectorySegment, String name) {
        try {
            File outputDir = new File("../out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
            for (int i = 0; i <= 200; i++) {
                double time = i / 200.0 * trajectorySegment.duration();
                Pose2d pose = trajectorySegment.get(time);
                Pose2d velocity = trajectorySegment.velocity(time);
                Pose2d acceleration = trajectorySegment.acceleration(time);
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
                Pose2d pose = trajectory.get(time);
                Pose2d velocity = trajectory.velocity(time);
                Pose2d acceleration = trajectory.acceleration(time);
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
        Mat field = Imgcodecs.imread("../FtcDashboard/dash/src/assets/field.png");
        MatOverlay overlay = new MatOverlay(field);
        overlay.setScalingFactor(field.width() / 144.0);
        drawTrajectory(overlay, trajectory, new Scalar(0, 255, 0));
        for (double t = 0; t <= trajectory.duration(); t += 0.25) {
            Vector2d pos = trajectory.get(t).pos();
            overlay.fillCircle(new Point(-pos.y() + 72, -pos.x() + 72), 0.5, new Scalar(0, 255, 255));
        }
        File outputDir = new File("../out");
        outputDir.mkdirs();
        Imgcodecs.imwrite(new File(outputDir, name + ".png").getAbsolutePath(), field);
    }

    public static void drawTrajectory(Overlay overlay, Trajectory trajectory, Scalar color) {
        for (TrajectorySegment motionSegment : trajectory.getSegments()) {
            if (motionSegment instanceof PathTrajectorySegment) {
                for (Path path : ((PathTrajectorySegment) motionSegment).getPaths()) {
                    drawPath(overlay, path, color);
                }
            }
        }
    }

    public static void drawPath(Overlay overlay, Path path, Scalar color) {
        ParametricCurve curve = path.getParametricCurve();
        if (curve instanceof LineSegment) {
            LineSegment line = (LineSegment) curve;
            overlay.strokeLine(new Point(-line.start().y() + 72, -line.start().x() + 72), new Point(-line.end().y() + 72, -line.end().x() + 72), color, 8);
        } else if (curve instanceof QuinticSplineSegment) {
            QuinticSplineSegment spline = (QuinticSplineSegment) curve;
            Point lastPoint = new Point(-spline.getY().getF() + 72, -spline.getX().getF() + 72);
            for (int i = 1; i <= 100; i++) {
                double t = i / 100.0 * spline.length();
                Vector2d pos = spline.get(t);
                Point point = new Point(-pos.y() + 72, -pos.x() + 72);
                overlay.strokeLine(lastPoint, point, color, 8);
                lastPoint = point;
            }
        }
    }
}
