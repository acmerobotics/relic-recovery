package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

import org.junit.Test;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by ryanbrott on 11/14/17.
 */

public class LinearSegmentTest {

    @Test
    public void testLinearSegment() throws IOException {
        LinearSegment linearSegment = new LinearSegment(new Pose2d(1, 1, Math.PI / 4), new Vector2d(2, 2));

        FileWriter fileWriter = new FileWriter(new File("/Users/ryanbrott/Desktop/path.csv"));
        fileWriter.write("timestamp,x,y\n");
        for (int i = 0; i < 1000; i++) {
            double time = linearSegment.duration() * i / 1000.0;
            Pose2d pose = linearSegment.getPose(time);
            fileWriter.write(time + "," + pose.x() + "," + pose.y() + "\n");
        }
        fileWriter.close();
    }
}
