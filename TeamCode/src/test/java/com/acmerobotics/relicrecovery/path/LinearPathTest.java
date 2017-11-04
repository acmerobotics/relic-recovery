package com.acmerobotics.relicrecovery.path;

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
            new Vector2d(0, 0),
            new Vector2d(2, 2));
    private LinearPath simplePath = new LinearPath(Arrays.asList(
            new Vector2d(0, 0),
            new Vector2d(2, 2),
            new Vector2d(3, 1),
            new Vector2d(4, 2)
    ));

    @Test
    public void testSegmentContains() {
        assertTrue(simpleSeg.contains(new Vector2d(1, 1)));
    }

    @Test
    public void testSegmentPosition() {
        assertTrue(simpleSeg.getPoint(0.5).equals(new Vector2d(1, 1)));
        assertEquals(0.5, simpleSeg.getPosition(new Vector2d(1, 1)), 0.0001);
        assertTrue(simpleSeg.getBoundedPoint(1.5).equals(simpleSeg.end()));
    }

    @Test
    public void testSegmentLength() {
        assertEquals(Math.sqrt(8), simpleSeg.length(), 0.0001);
    }

    @Test
    public void testSegmentClosestPoint() {
        assertEquals(0.25, simpleSeg.getClosestPositionOnPath(new Vector2d(1, 0)), 0.0001);
    }

    @Test
    public void testPathPosition() {
        assertTrue(simplePath.getPoint(0.5).equals(new Vector2d(2, 2)));
        assertEquals(0.75, simplePath.getPosition(new Vector2d(3, 1)), 0.0001);
    }

    @Test
    public void testPathLength() {
        assertEquals(Math.sqrt(32), simplePath.length(), 0.0001);
    }

    @Test
    public void testPathDistance() {
        LinearPath.DistanceReport report = simplePath.getDistanceReport(new Vector2d(0, 1));
        assertEquals(Math.hypot(0.5, 0.5), report.distance, 0.0001);
        assertTrue(report.pathPoint.equals(new Vector2d(0.5, 0.5)));
    }

}
