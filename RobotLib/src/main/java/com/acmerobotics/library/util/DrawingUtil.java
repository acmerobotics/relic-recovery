package com.acmerobotics.library.util;


import com.acmerobotics.dashboard.canvas.Canvas;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.path.LineSegment;
import com.acmerobotics.splinelib.path.ParametricCurve;
import com.acmerobotics.splinelib.path.Path;
import com.acmerobotics.splinelib.path.QuinticPolynomial;
import com.acmerobotics.splinelib.path.QuinticSplineSegment;
import com.acmerobotics.splinelib.trajectory.PathTrajectorySegment;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.acmerobotics.splinelib.trajectory.TrajectorySegment;

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
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).plus(wheelPose.pos()));
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
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).plus(wheelPose.pos()));
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
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).plus(wheelPose.pos()));
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
            wheelPath.set(i, wheelPath.get(i).rotated(wheelPose.heading()).plus(wheelPose.pos()));
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
            robotPath.set(i, robotPath.get(i).rotated(robotPose.heading()).plus(robotPose.pos()));
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
            Vector2d adjustedOrigin = wheelOrigins.get(i).rotated(robotPose.heading()).plus(robotPose.pos());
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
        for (TrajectorySegment motionSegment : trajectory.getSegments()) {
            if (motionSegment instanceof PathTrajectorySegment) {
                PathTrajectorySegment pathTrajectorySegment = (PathTrajectorySegment) motionSegment;
                for (Path path : pathTrajectorySegment.getPaths()) {
                    drawParametricCurve(canvas, path.getParametricCurve());
                }
            }
        }
    }

    public static void drawParametricCurve(Canvas canvas, ParametricCurve parametricCurve) {
        if (parametricCurve instanceof LineSegment) {
            LineSegment line = (LineSegment) parametricCurve;
            canvas.strokeLine(line.get(0.0).x(), line.get(0.0).y(), line.get(line.length()).x(), line.get(line.length()).x());
        } else if (parametricCurve instanceof QuinticSplineSegment) {
            QuinticSplineSegment spline = (QuinticSplineSegment) parametricCurve;
            QuinticPolynomial x = spline.getX();
            QuinticPolynomial y = spline.getY();
            canvas.strokeSpline(x.getA(), x.getB(), x.getC(), x.getD(), x.getE(), x.getF(),
                                y.getA(), y.getB(), y.getC(), y.getD(), y.getE(), y.getF());
        }
    }
}
