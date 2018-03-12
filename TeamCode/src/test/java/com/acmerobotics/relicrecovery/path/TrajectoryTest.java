package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;

import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static org.junit.Assert.assertEquals;

public class TrajectoryTest {
    @Ignore
    @Test
    public void simulatePathFollowing() throws IOException {
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(0, 0, 0))
                .turn(3 * Math.PI / 8)
                .lineTo(new Vector2d(1, 1))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(2, 3))
                .lineTo(new Vector2d(2, 5))
                .build();
        TrajectoryFollower follower = new TrajectoryFollower(
                new PIDFCoefficients(-0.01, 0, 0, 1, 0),
                new PIDFCoefficients(-0.1, 0, 0, 1, 0),
                new PIDFCoefficients(-0.1, 0, 0, 1, 0));
        follower.follow(trajectory);
        double timestamp = TimestampedData.getCurrentTime();
        Pose2d estimatedPose = trajectory.getPose(0);
        File dataFile = new File("path.csv");
        FileWriter writer = new FileWriter(dataFile);
        writer.write("x,y,heading\n");
        while (follower.isFollowingTrajectory(timestamp)) {
            Pose2d update = follower.update(estimatedPose, timestamp);
            Vector2d vector = new Vector2d(0.01 * update.x() + 0.01 * (Math.random() - 0.5),
                    0.01 * update.y() + 0.01 * (Math.random() - 0.5));
            Pose2d distances = new Pose2d(vector.rotated(estimatedPose.heading()),
                    0.01 * update.heading() + 0.01 * (Math.random() - 0.5));
            estimatedPose = estimatedPose.added(distances);
            System.out.println(estimatedPose);
            writer.write(String.format("%f,%f,%f\n", estimatedPose.x(), estimatedPose.y(), estimatedPose.heading()));
            writer.flush();
            timestamp += 0.01;
        }
        writer.close();

        Pose2d endPose = trajectory.getPose(trajectory.duration() - 0.0001);
        assertEquals(endPose.x(), estimatedPose.x(), 0.1);
        assertEquals(endPose.y(), estimatedPose.y(), 0.1);
        assertEquals(endPose.heading(), estimatedPose.heading(), 0.1);
    }
}
