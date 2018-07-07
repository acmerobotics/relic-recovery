package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.motion.PIDFCoefficients;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.TrajectoryFollower;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.trajectory.Trajectory;

import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static org.junit.Assert.assertEquals;

public class TrajectoryFollowerTest {
    @Ignore
    @Test
    public void simulateNoisyTrajectoryFollowing() throws IOException {
//        Trajectory trajectory = AutoPaths.trajectoryBuilder(new Pose2d(0, 0, 0))
//                .turn(3 * Math.PI / 8)
//                .lineTo(new Vector2d(1, 1))
//                .turn(-Math.PI / 2)
//                .lineTo(new Vector2d(2, 3))
//                .lineTo(new Vector2d(2, 5))
//                .build();
        Trajectory trajectory = AutoPaths.trajectoryBuilder(new Pose2d(48, -48, Math.PI))
                .beginComposite()
                .lineTo(new Vector2d(36, -48))
                .splineTo(new Pose2d(0, -12, 3 * Math.PI / 4))
                .closeComposite()
//                .beginComposite()
                .splineTo(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -60))
//                .closeComposite()
                .splineTo(new Pose2d(24, -12, Math.PI / 4))
//                .beginComposite()
                .splineTo(new Pose2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -44, Math.PI / 2))
                .lineTo(new Vector2d(12 - AutoPaths.CRYPTO_COL_WIDTH, -60))
//                .closeComposite()
                .splineTo(new Pose2d(16, 0, 3 * Math.PI / 8))
//                .beginComposite()
                .splineTo(new Pose2d(12, -44, Math.PI / 2))
                .lineTo(new Vector2d(12, -60))
//                .closeComposite()
                .build();
        TrajectoryFollower follower = new TrajectoryFollower(
                new PIDFCoefficients(-0.01, 0, 0, 1, 0),
                new PIDFCoefficients(-0.1, 0, 0, 1, 0),
                new PIDFCoefficients(-0.1, 0, 0, 1, 0));
        follower.follow(trajectory);
        double timestamp = TimestampedData.getCurrentTime();
        Pose2d estimatedPose = trajectory.get(0);
        File dataFile = new File("trajectorySimulation.csv");
        FileWriter writer = new FileWriter(dataFile);
        writer.write("x,y,heading\n");
        while (follower.isFollowingTrajectory(timestamp)) {
            Pose2d update = follower.update(estimatedPose, timestamp);
            Vector2d vector = new Vector2d(0.01 * update.x() + 0.01 * (Math.random() - 0.5),
                    0.01 * update.y() + 0.01 * (Math.random() - 0.5));
            Pose2d distances = new Pose2d(vector.rotated(estimatedPose.heading()),
                    0.01 * update.heading() + 0.01 * (Math.random() - 0.5));
            estimatedPose = estimatedPose.plus(distances);
            System.out.println(estimatedPose);
            writer.write(String.format("%f,%f,%f\n", estimatedPose.x(), estimatedPose.y(), estimatedPose.heading()));
            writer.flush();
            timestamp += 0.01;
        }
        writer.close();

        Pose2d endPose = trajectory.get(trajectory.duration() - 0.0001);
        assertEquals(endPose.x(), estimatedPose.x(), 0.2);
        assertEquals(endPose.y(), estimatedPose.y(), 0.2);
        assertEquals(endPose.heading(), estimatedPose.heading(), 0.2);
    }


    @Ignore
    @Test
    public void testTrajectoryFollower() throws IOException {
        Trajectory trajectory = AutoPaths.trajectoryBuilder(new Pose2d(60, 0, Math.PI))
                .splineTo(new Pose2d(24, -24, Math.PI))
                .splineTo(new Pose2d(12, -52, Math.PI / 2))
                .build();
        TrajectoryFollower follower = new TrajectoryFollower(
                new PIDFCoefficients(0, 0, 0, 1, 0),
                new PIDFCoefficients(0, 0, 0, 1, 0),
                new PIDFCoefficients(0, 0, 0, 1, 0));
        double timestamp = follower.follow(trajectory);
        Pose2d estimatedPose = trajectory.start();
        File dataFile = new File("trajectoryFollower.csv");
        FileWriter writer = new FileWriter(dataFile);
        writer.write("x,y,heading\n");
        while (follower.isFollowingTrajectory(timestamp)) {
            Pose2d update = follower.update(estimatedPose, timestamp);
            Vector2d vector = new Vector2d(0.01 * update.x(), 0.01 * update.y());
            Pose2d distances = new Pose2d(vector.rotated(estimatedPose.heading()), 0.01 * update.heading());
            estimatedPose = estimatedPose.plus(distances);
            System.out.printf("%1$6.2f,%2$6.2f,%3$6.2f\n", distances.x() * 100, distances.y() * 100, update.heading());
//            System.out.println(estimatedPose);
            writer.write(String.format("%f,%f,%f\n", estimatedPose.x(), estimatedPose.y(), estimatedPose.heading()));
            writer.flush();
            timestamp += 0.01;
        }
        writer.close();
    }
}
