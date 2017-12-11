package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.drive.PathFollower;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

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
                .lineTo(new Pose2d(1, 1, 0))
                .lineTo(new Pose2d(2, 1, 0))
                .turn(Math.PI / 2)
                .build();
        PathFollower follower = new PathFollower(DriveConstants.HEADING_COEFFS, DriveConstants.AXIAL_COEFFS, DriveConstants.LATERAL_COEFFS);
        follower.follow(path);
        long timestamp = System.currentTimeMillis();
        Pose2d estimatedPose = new Pose2d(0, 0, 0);
        File dataFile = new File("/Users/ryanbrott/Desktop/test.csv");
        FileWriter writer = new FileWriter(dataFile);
        writer.write("x,y,heading\n");
        while (follower.isFollowingPath(timestamp)) {
            Pose2d update = follower.update(estimatedPose, timestamp);
            Pose2d distances = new Pose2d(0.01 * update.x() / DriveConstants.AXIAL_COEFFS.v,
                    0.01 * update.y() / DriveConstants.LATERAL_COEFFS.v,
                    0.01 * update.heading() / DriveConstants.HEADING_COEFFS.v);
            estimatedPose = estimatedPose.added(distances);
            System.out.println(estimatedPose);
            writer.write(String.format("%f,%f,%f\n", estimatedPose.x(), estimatedPose.y(), estimatedPose.heading()));
            writer.flush();
            timestamp += 10;
        }
        writer.close();
    }
}
