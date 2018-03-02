package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path.parametric.CompositePath;
import com.acmerobotics.relicrecovery.path.parametric.ParametricPath;

import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class SplinePathTest {
    @Ignore
    @Test
    public void testSplineGeneration() {
        ParametricPath spline = CompositePath.fitSpline(
                new Pose2d(0, 0, Math.PI / 4),
                new Pose2d(24, 12, 0),
                new Pose2d(36, 8, Math.PI / 4),
                new Pose2d(48, 4, 3 * Math.PI / 4));
        writeParametricPath(spline, "simpleSpline");

        TrajectorySegment motionSegment = new ParametricSegment(spline);
        writePathMotionSegment(motionSegment, "simpleMotionSegment");
    }

    public static void writeParametricPath(ParametricPath path, String name) {
        try {
            File outputDir = new File("./out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
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
            File outputDir = new File("./out");
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
}
