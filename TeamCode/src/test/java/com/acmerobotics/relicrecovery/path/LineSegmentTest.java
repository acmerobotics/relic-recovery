package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

import static org.junit.Assert.assertEquals;

/**
 * Created by ryanbrott on 11/14/17.
 */

public class LineSegmentTest {

    @Test
    public void testLinearSegment() throws IOException {
        LineSegment lineSegment = new LineSegment(new Pose2d(1, 1, Math.PI / 4), new Pose2d(2, 2, Math.PI / 4));

        FileWriter fileWriter = new FileWriter(new File("/Users/ryanbrott/Desktop/path.csv"));
        fileWriter.write("timestamp,x,y\n");
        for (int i = 0; i < 1000; i++) {
            double time = lineSegment.duration() * i / 1000.0;
            Pose2d pose = lineSegment.getPose(time);
            fileWriter.write(time + "," + pose.x() + "," + pose.y() + "\n");
        }
        fileWriter.close();
    }

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
