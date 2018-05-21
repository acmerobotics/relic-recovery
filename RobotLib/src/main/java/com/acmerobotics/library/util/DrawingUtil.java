package com.acmerobotics.library.util;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.path.ParametricSegment;
import com.acmerobotics.library.path.Trajectory;
import com.acmerobotics.library.path.TrajectorySegment;
import com.acmerobotics.library.path.parametric.CompositePath;
import com.acmerobotics.library.path.parametric.LinePath;
import com.acmerobotics.library.path.parametric.ParametricPath;
import com.acmerobotics.library.path.parametric.SplinePath;

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

    public static void drawTrajectory(Canvas canvas, Trajectory trajectory) {
        canvas.setStrokeWidth(3);
        for (TrajectorySegment motionSegment : trajectory.segments()) {
            if (motionSegment instanceof ParametricSegment) {
                drawParametricPath(canvas, ((ParametricSegment) motionSegment).path());
            }
        }
    }

    public static void drawParametricPath(Canvas canvas, ParametricPath path) {
        if (path instanceof CompositePath) {
            for (ParametricPath subPath : ((CompositePath) path).segments()) {
                drawParametricPath(canvas, subPath);
            }
        } else if (path instanceof LinePath) {
            LinePath line = (LinePath) path;
            canvas.strokeLine(line.start().x(), line.start().y(), line.end().x(), line.end().y());
        } else if (path instanceof SplinePath) {
            SplinePath spline = (SplinePath) path;
            canvas.strokeSpline(spline.knotDistance(), spline.xOffset(), spline.yOffset(), spline.headingOffset(),
                    spline.a(), spline.b(), spline.c(), spline.d(), spline.e());
        }
    }
}
