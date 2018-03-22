package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class TrajectoryBuilderTest {
    @Test
    public void testPiecewisePath() {
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(48, -48, Math.PI))
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
