package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Vector2d;

import org.junit.Test;

import java.util.Arrays;

import static junit.framework.Assert.*;

/**
 * @author Ryan
 */

public class LinearPathTest {

    private LinearPath.Segment simpleSeg = new LinearPath.Segment(
            new LinearPath.Waypoint(new Vector2d(0, 0)),
            new LinearPath.Waypoint(new Vector2d(2, 2)));
    private LinearPath simplePath = new LinearPath(Arrays.asList(
            new LinearPath.Waypoint(new Vector2d(0, 0)),
            new LinearPath.Waypoint(new Vector2d(2, 2)),
            new LinearPath.Waypoint(new Vector2d(3, 1)),
            new LinearPath.Waypoint(new Vector2d(4, 2))
    ));

    @Test
    public void testSegmentContains() {
        assertTrue("Segment contains doesn't work", simpleSeg.contains(new Vector2d(1, 1)));
    }

    @Test
    public void testSegmentPosition() {
        assertTrue("Segment get point doesn't work", simpleSeg.getPoint(0.5).equals(new Vector2d(1, 1)));
        assertEquals("Segment get position doesn't work", 0.5, simpleSeg.getPosition(new Vector2d(1, 1)), 0.0001);
        assertTrue("Segment bounded position doesn't work", simpleSeg.getBoundedPoint(1.5).equals(simpleSeg.end));
    }

    @Test
    public void testSegmentLength() {
        assertEquals("Segment length doesn't work", Math.sqrt(8), simpleSeg.length(), 0.0001);
    }

    @Test
    public void testSegmentClosestPoint() {
        assertEquals("Segment closest point doesn't work", 0.25, simpleSeg.getClosestPositionOnPath(new Vector2d(1, 0)), 0.0001);
    }

    @Test
    public void testPathPosition() {
        assertTrue("Path get point doesn't work", simplePath.getPoint(0.5).equals(new Vector2d(2, 2)));
        assertEquals("Path get position doesn't work", 0.75, simplePath.getPosition(new Vector2d(3, 1)), 0.0001);
    }

    @Test
    public void testPathLength() {
        assertEquals("Path length doesn't work", Math.sqrt(32), simplePath.length(), 0.0001);
    }

    @Test
    public void testPathDistance() {
        LinearPath.DistanceReport report = simplePath.getDistanceReport(new Vector2d(0, 1));
        assertEquals("Path distance doesn't work", Math.hypot(0.5, 0.5), report.distance, 0.0001);
        assertTrue("Path closest point doesn't work", report.pathPoint.equals(new Vector2d(0.5, 0.5)));
    }

}
