package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

import org.junit.Test;

import static org.junit.Assert.assertEquals;

/**
 * Created by ryanbrott on 11/14/17.
 */

public class LineSegmentTest {

    @Test
    public void testProfileBounds() {
        LineSegment lineSegment = new LineSegment(new Pose2d(1, 1, Math.PI / 4), new Pose2d(2, 2, Math.PI / 4));
        assertEquals(lineSegment.getPose(0), new Pose2d(1, 1, Math.PI / 4));
        assertEquals(lineSegment.getPose(lineSegment.duration()), new Pose2d(2, 2, Math.PI / 4));
    }

    @Test
    public void testProfileBoundsNegated() {
        LineSegment lineSegment = new LineSegment(new Pose2d(-1, -1, -3 * Math.PI / 4), new Pose2d(-2, -2, -3 * Math.PI / 4));
        assertEquals(lineSegment.getPose(0), new Pose2d(-1, -1, -3 * Math.PI / 4));
        assertEquals(lineSegment.getPose(lineSegment.duration()), new Pose2d(-2, -2, -3 * Math.PI / 4));
    }
}
