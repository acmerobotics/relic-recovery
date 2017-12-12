package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.PathFollower;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;

import org.junit.Ignore;
import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * @author Ryan
 */

public class PathTest {

    @Test
    public void testBasicPathCreation() {
        List<PathSegment> expectedSegments = Arrays.asList(
                new LineSegment(new Pose2d(0, 0, 0), new Pose2d(2, 0, 0)),
                new PointTurn(new Pose2d(2, 0), Math.PI / 2),
                new LineSegment(new Pose2d(2, 0, Math.PI / 2), new Pose2d(2, 2, Math.PI / 2)),
                new PointTurn(new Pose2d(2, 2), Math.PI / 2)
        );

        Path path = Path.createFromPoses(Arrays.asList(
                new Pose2d(0, 0),
                new Pose2d(2, 0),
                new Pose2d(2, 2, Math.PI)
        ));

        assertEquals(expectedSegments, path.getSegments());
    }

    @Test
    public void testEmptyTurnPathCreation() {
        List<PathSegment> expectedSegments = Arrays.asList(
                new LineSegment(new Pose2d(0, 0, 0), new Pose2d(2, 0, 0)),
                new LineSegment(new Pose2d(2, 0, 0), new Pose2d(4, 0, 0))
        );

        Path path = Path.createFromPoses(Arrays.asList(
                new Pose2d(0, 0),
                new Pose2d(2, 0),
                new Pose2d(4, 0)
        ));

        assertEquals(expectedSegments, path.getSegments());
    }

    @Ignore
    @Test
    public void simulatePathFollowing() throws IOException {
        Path path = new PathBuilder(new Pose2d(0, 0, 0))
                .turn(3 * Math.PI / 8)
                .lineTo(new Vector2d(1, 1))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(2, 3))
                .lineTo(new Vector2d(2, 5))
                .build();
        PathFollower follower = new PathFollower(
                new PIDFCoefficients(-0.01, 0, 0, 1, 0),
                new PIDFCoefficients(-0.1, 0, 0, 1, 0),
                new PIDFCoefficients(-0.1, 0, 0, 1, 0));
        follower.follow(path);
        long timestamp = System.currentTimeMillis();
        Pose2d estimatedPose = path.getPose(0);
        File dataFile = new File("path.csv");
        FileWriter writer = new FileWriter(dataFile);
        writer.write("x,y,heading\n");
        while (follower.isFollowingPath(timestamp)) {
            Pose2d update = follower.update(estimatedPose, timestamp);
            Vector2d vector = new Vector2d(0.01 * update.x() + 0.01 * (Math.random() - 0.5),
                    0.01 * update.y() + 0.01 * (Math.random() - 0.5));
            Pose2d distances = new Pose2d(vector.rotated(estimatedPose.heading()),
                    0.01 * update.heading() + 0.01 * (Math.random() - 0.5));
            estimatedPose = estimatedPose.added(distances);
            System.out.println(estimatedPose);
            writer.write(String.format("%f,%f,%f\n", estimatedPose.x(), estimatedPose.y(), estimatedPose.heading()));
            writer.flush();
            timestamp += 10;
        }
        writer.close();

        Pose2d endPose = path.getPose(path.duration() - 0.0001);
        assertEquals(endPose.x(), estimatedPose.x(), 0.1);
        assertEquals(endPose.y(), estimatedPose.y(), 0.1);
        assertEquals(endPose.heading(), estimatedPose.heading(), 0.1);
    }
}
