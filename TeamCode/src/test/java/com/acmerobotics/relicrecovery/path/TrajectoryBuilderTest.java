package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class TrajectoryBuilderTest {
    @Test
    public void testPiecewisePath() {
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(60, 0, Math.PI))
                .splineThrough(new Pose2d(24, -24, Math.PI), new Pose2d(12, -52, Math.PI / 2))
                .build();
        writeTrajectory(trajectory, "piecewise");
    }

    public static void writeTrajectory(Trajectory trajectory, String name) {
        try {
            FileWriter writer = new FileWriter(new File(name + ".csv"));
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
}
