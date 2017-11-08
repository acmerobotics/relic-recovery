package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;

import org.junit.Test;

import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * @author Ryan
 */

public class PathTest {

    @Test
    public void testPathCreationFromPoses() {
        List<PathSegment> expectedSegments = Arrays.asList(
                new LinearSegment(new Vector2d(0, 0), new Vector2d(2, 0)),
                new PointTurn(new Vector2d(2, 0), Math.PI / 2),
                new LinearSegment(new Vector2d(2, 0), new Vector2d(2, 2)),
                new PointTurn(new Vector2d(2, 2), Math.PI / 2)
        );

        Path path = Path.createFromPoses(Arrays.asList(
                new Pose2d(0, 0),
                new Pose2d(2, 0),
                new Pose2d(2, 2, Math.PI)
        ));

        assertEquals(expectedSegments, path.getSegments());
    }

}
