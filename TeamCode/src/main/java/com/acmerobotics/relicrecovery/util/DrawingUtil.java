package com.acmerobotics.relicrecovery.util;

import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.path.LineSegment;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathSegment;

import java.util.Arrays;
import java.util.List;

/**
 * Created by ryanbrott on 1/17/18.
 */

public class DrawingUtil {
    private static List<Vector2d> getFrontLeftWheelContour(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, -2),
                new Vector2d(3, -2),
                new Vector2d(3, 2),
                new Vector2d(-3, 2),
                new Vector2d(-3, -2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getFrontLeftWheelPattern(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, -2),
                new Vector2d(-1, 2),
                new Vector2d(1, 2),
                new Vector2d(-1, -2),
                new Vector2d(1, -2),
                new Vector2d(3, 2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getRearLeftWheelContour(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, 2),
                new Vector2d(3, 2),
                new Vector2d(3, -2),
                new Vector2d(-3, -2),
                new Vector2d(-3, 2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }

    private static List<Vector2d> getRearLeftWheelPattern(Pose2d wheelPose) {
        List<Vector2d> wheelPath = Arrays.asList(
                new Vector2d(-3, 2),
                new Vector2d(-1, -2),
                new Vector2d(1, -2),
                new Vector2d(-1, 2),
                new Vector2d(1, 2),
                new Vector2d(3, -2)
        );
        for (int i = 0; i < wheelPath.size(); i++) {
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).added(wheelPose.pos()));
        }
        return wheelPath;
    }

    public static void drawMecanumRobot(Canvas canvas, Pose2d robotPose) {
        canvas.setStrokeWidth(2);
        // robot body
        List<Vector2d> robotPath = Arrays.asList(
                new Vector2d(9, 9),
                new Vector2d(-9, 9),
                new Vector2d(-9, -9),
                new Vector2d(9, -9),
                new Vector2d(9, 0),
                new Vector2d(3, 0),
                new Vector2d(9, 0),
                new Vector2d(9, 9)
        );
        for (int i = 0; i < robotPath.size(); i++) {
            robotPath.set(i, robotPath.get(i).rotated(robotPose.heading()).added(robotPose.pos()));
        }
        drawVectorPolyline(canvas, robotPath);

        // robot wheels
        List<Vector2d> wheelOrigins = Arrays.asList(
                new Vector2d(4.5, 5.5),
                new Vector2d(-4.5, 5.5),
                new Vector2d(-4.5, -5.5),
                new Vector2d(4.5, -5.5)
        );
        for (int i = 0; i < wheelOrigins.size(); i++) {
            Vector2d adjustedOrigin = wheelOrigins.get(i).rotated(robotPose.heading()).added(robotPose.pos());
            if (i % 2 == 0) {
                drawVectorPolyline(canvas, getFrontLeftWheelContour(new Pose2d(adjustedOrigin, robotPose.heading())));
                drawVectorPolyline(canvas, getFrontLeftWheelPattern(new Pose2d(adjustedOrigin, robotPose.heading())));
            } else {
                drawVectorPolyline(canvas, getRearLeftWheelContour(new Pose2d(adjustedOrigin, robotPose.heading())));
                drawVectorPolyline(canvas, getRearLeftWheelPattern(new Pose2d(adjustedOrigin, robotPose.heading())));
            }
        }
    }

    private static void drawVectorPolyline(Canvas canvas, List<Vector2d> vectors) {
        double[] xCoords = new double[vectors.size()];
        double[] yCoords = new double[vectors.size()];
        for (int i = 0; i < xCoords.length; i++) {
            xCoords[i] = vectors.get(i).x();
            yCoords[i] = vectors.get(i).y();
        }
        canvas.strokePolyline(xCoords, yCoords);
    }

    public static void drawPath(Canvas canvas, Path path) {
        canvas.setStrokeWidth(3);
        for (PathSegment segment : path.getSegments()) {
            if (segment instanceof LineSegment) {
                LineSegment lineSegment = (LineSegment) segment;
                canvas.strokeLine(lineSegment.start().x(), lineSegment.start().y(),
                        lineSegment.end().x(), lineSegment.end().y());
            }
        }
    }
}
