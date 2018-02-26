package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
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
        writeSpline(splineSegment, "simpleTest");
    }

    public static void writeSpline(SplineSegment spline, String name) {
        try {
            File outputDir = new File("./out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, heading\n");
            for (int i = 0; i <= 200; i++) {
                double displacement = i / 200.0 * spline.length();
                Pose2d pose = spline.getPose(displacement);
                writer.write(String.format("%f, %f, %f, %f\n", displacement, pose.x(), pose.y(), pose.heading()));
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
