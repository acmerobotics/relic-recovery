package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class TrajectoryBuilderTest {
    @Test
    public void testPiecewisePath() {
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0))
                .beginComposite()
                .lineTo(new Vector2d(12, 0))
                .splineThrough(new Pose2d(16, 4, -Math.PI / 2))
                .lineTo(new Vector2d(16, 18))
                .build();
        writeTrajectory(trajectory, "piecewise");
    }

    public static void writeTrajectory(Trajectory trajectory, String name) {
        try {
            File outputDir = new File("./out");
            outputDir.mkdirs();
            FileWriter writer = new FileWriter(new File(outputDir,name + ".csv"));
            writer.write("t, x, y, theta, vx, vy, omega, ax, ay, alpha\n");
            System.out.println("starting duration: " + trajectory.duration());
            double intervalWidth = 1 / 200.0 * trajectory.duration();
            for (int i = 0; i <= 200; i++) {
                double time = i * intervalWidth;
                if (time > trajectory.duration()) {
                    System.out.println("finished early");
                    break;
                }
                if (i == 50) {
                    trajectory.trimRemainingDistance(time);
                    System.out.println("trimmed");
                    System.out.println("new duration: " + trajectory.duration());
                }
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
}
