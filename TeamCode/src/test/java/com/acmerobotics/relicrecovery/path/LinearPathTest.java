package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;

import org.junit.Test;

import java.util.Arrays;

import static junit.framework.Assert.assertEquals;
import static junit.framework.Assert.assertTrue;

/**
 * @author Ryan
 */

public class LinearPathTest {

    private LinearPath.Segment simpleSeg = new LinearPath.Segment(
            new Pose2d(0, 0),
            new Pose2d(2, 2));
    private LinearPath simplePath = new LinearPath(Arrays.asList(
            new Pose2d(0, 0),
            new Pose2d(2, 2),
            new Pose2d(3, 1),
            new Pose2d(4, 2)
    ));

    @Test
    public void testSegmentContains() {
        assertTrue(simpleSeg.contains(new Vector2d(1, 1)));
    }

    @Test
    public void testSegmentPosition() {
        assertEquals(new Vector2d(1, 1), simpleSeg.getPose(0.5).pos());
        assertEquals(0.5, simpleSeg.getPosition(new Vector2d(1, 1)), Vector2d.EPSILON);
        assertEquals(simpleSeg.end(), simpleSeg.getBoundedPose(1.5));
    }

    @Test
    public void testSegmentLength() {
        assertEquals(Math.sqrt(8), simpleSeg.length(), Vector2d.EPSILON);
    }

    @Test
    public void testSegmentClosestPoint() {
        assertEquals(0.25, simpleSeg.getClosestPositionOnPath(new Vector2d(1, 0)), Vector2d.EPSILON);
    }

    @Test
    public void testPathPosition() {
        assertEquals(new Vector2d(2, 2), simplePath.getPose(0.5).pos());
        assertEquals(0.75, simplePath.getPosition(new Vector2d(3, 1)), Vector2d.EPSILON);
    }

    @Test
    public void testPathLength() {
        assertEquals(Math.sqrt(32), simplePath.length(), Vector2d.EPSILON);
    }

    @Test
    public void testPathDistance() {
        LinearPath.DistanceReport report = simplePath.getDistanceReport(new Vector2d(0, 1));
        assertEquals(Math.hypot(0.5, 0.5), report.distance, Vector2d.EPSILON);
        assertEquals(new Vector2d(0.5, 0.5), report.pathPoint);
    }

}
