package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.path.parametric.CompositePath;
import com.acmerobotics.relicrecovery.path.parametric.LinePath;
import com.acmerobotics.relicrecovery.path.parametric.ParametricPath;
import com.acmerobotics.relicrecovery.path.parametric.SplinePath;

import java.util.ArrayList;
import java.util.List;

public class PathBuilder {
    private Pose2d currentPose;
    private List<PathMotionSegment> motionSegments;
    private List<ParametricPath> compositeSegments;
    private boolean composite;

    public PathBuilder(Pose2d pose) {
        currentPose = pose;
        motionSegments = new ArrayList<>();
        compositeSegments = new ArrayList<>();
    }

    public PathBuilder lineTo(Vector2d pos) {
        Pose2d pose = new Pose2d(pos, currentPose.heading());
        ParametricPath path = new LinePath(currentPose, pose);
        if (composite) {
            compositeSegments.add(path);
        } else {
            motionSegments.add(new ParametricMotionSegment(path));
        }
        currentPose = pose;
        return this;
    }

    public PathBuilder turn(double angle) {
        return turnTo(Angle.norm(currentPose.heading() + angle));
    }

    public PathBuilder turnTo(double heading) {
        PointTurn pointTurn = new PointTurn(currentPose, heading);
        motionSegments.add(pointTurn);
        currentPose = new Pose2d(currentPose.pos(), heading);
        return this;
    }

    public PathBuilder forward(double distance) {
        return lineTo(currentPose.pos().added(new Vector2d(
                distance * Math.cos(currentPose.heading()),
                distance * Math.sin(currentPose.heading())
        )));
    }

    public PathBuilder back(double distance) {
        return forward(-distance);
    }

    public PathBuilder strafeLeft(double distance) {
        return lineTo(currentPose.pos().added(new Vector2d(
                distance * Math.cos(currentPose.heading() + Math.PI / 2),
                distance * Math.sin(currentPose.heading() + Math.PI / 2)
        )));
    }

    public PathBuilder strafeRight(double distance) {
        return strafeLeft(-distance);
    }

    public PathBuilder splineThrough(SplinePath.Type type, Pose2d... waypoints) {
        CompositePath spline = CompositePath.fitSpline(type, waypoints);
        if (composite) {
            compositeSegments.add(spline);
        } else {
            motionSegments.add(new ParametricMotionSegment(spline));
        }
        return this;
    }

    public PathBuilder splineThrough(Pose2d... waypoints) {
        CompositePath spline = CompositePath.fitSpline(waypoints);
        if (composite) {
            compositeSegments.add(spline);
        } else {
            motionSegments.add(new ParametricMotionSegment(spline));
        }
        return this;
    }

    public PathBuilder waitFor(double seconds) {
        motionSegments.add(new WaitSegment(currentPose, seconds));
        return this;
    }

    public void beginComposite() {
        if (composite) {
            closeComposite();
        }
        composite = true;
    }

    public void closeComposite() {
        composite = false;
        motionSegments.add(new ParametricMotionSegment(new CompositePath(compositeSegments)));
        compositeSegments = new ArrayList<>();
    }

    public Path build() {
        if (composite) {
            closeComposite();
        }
        return new Path(motionSegments);
    }
}
