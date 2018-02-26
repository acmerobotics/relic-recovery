package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path2.ParametricMotionSegment;
import com.acmerobotics.relicrecovery.path2.PathMotionSegment;
import com.acmerobotics.relicrecovery.path2.SplineSegment;

import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class SplineTest {
    @Ignore
    @Test
    public void testSplineGeneration() {
        SplineSegment splineSegment = new SplineSegment(new Pose2d(0, 0, Math.PI / 4), new Pose2d(24, 12, 0));
        writeSplineSegment(splineSegment, "simpleSplineSegment");
        System.out.println(splineSegment.length());
        System.out.println(splineSegment.getPose(0.5));
        System.out.println(splineSegment.getDerivative(0.5));
        System.out.println(splineSegment.getSecondDerivative(0.5));

        PathMotionSegment motionSegment = new ParametricMotionSegment(splineSegment);
        writePathMotionSegment(motionSegment, "simpleMotionSegment");
    }

    public static void writeSplineSegment(SplineSegment splineSegment, String name) {
        try {
            File outputDir = new File("./out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, vx, vy, ax, ay\n");
            for (int i = 0; i <= 200; i++) {
                double displacement = i / 200.0 * splineSegment.length();
                Pose2d pose = splineSegment.getPose(displacement);
                Pose2d velocity = splineSegment.getDerivative(displacement);
                Pose2d acceleration = splineSegment.getSecondDerivative(displacement);
                writer.write(String.format("%f, %f, %f, %f, %f, %f, %f\n", displacement, pose.x(), pose.y(), velocity.x(), velocity.y(), acceleration.x(), acceleration.y()));
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    public static void writePathMotionSegment(PathMotionSegment motionSegment, String name) {
        try {
            File outputDir = new File("./out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, vx, vy, ax, ay\n");
            for (int i = 0; i <= 200; i++) {
                double time = i / 200.0 * motionSegment.duration();
                Pose2d pose = motionSegment.getPose(time);
                Pose2d velocity = motionSegment.getVelocity(time);
                Pose2d acceleration = motionSegment.getAcceleration(time);
                writer.write(String.format("%f, %f, %f, %f, %f, %f, %f\n", time, pose.x(), pose.y(), velocity.x(), velocity.y(), acceleration.x(), acceleration.y()));

            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
