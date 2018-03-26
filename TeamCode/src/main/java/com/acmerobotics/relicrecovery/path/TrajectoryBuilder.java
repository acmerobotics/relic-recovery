package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.path.parametric.CompositePath;
import com.acmerobotics.relicrecovery.path.parametric.LinePath;
import com.acmerobotics.relicrecovery.path.parametric.ParametricPath;
import com.acmerobotics.relicrecovery.path.parametric.SplinePath;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

public class TrajectoryBuilder {
    private Pose2d currentPose;
    private List<TrajectorySegment> motionSegments;
    private List<ParametricPath> compositeSegments;
    private boolean composite;

    public TrajectoryBuilder(Pose2d pose) {
        currentPose = pose;
        motionSegments = new ArrayList<>();
        compositeSegments = new ArrayList<>();
    }

    public TrajectoryBuilder lineTo(Vector2d pos) {
        Pose2d pose = new Pose2d(pos, currentPose.heading());
        ParametricPath path = new LinePath(currentPose, pose);
        if (composite) {
            compositeSegments.add(path);
        } else {
            motionSegments.add(new ParametricSegment(path));
        }
        currentPose = path.end();
        return this;
    }

    public TrajectoryBuilder turn(double angle) {
        return turnTo(Angle.norm(currentPose.heading() + angle));
    }

    public TrajectoryBuilder turnTo(double heading) {
        PointTurn pointTurn = new PointTurn(currentPose, heading);
        motionSegments.add(pointTurn);
        currentPose = new Pose2d(currentPose.pos(), heading);
        return this;
    }

    public TrajectoryBuilder forward(double distance) {
        return lineTo(currentPose.pos().added(new Vector2d(
                distance * Math.cos(currentPose.heading()),
                distance * Math.sin(currentPose.heading())
        )));
    }

    public TrajectoryBuilder back(double distance) {
        return forward(-distance);
    }

    public TrajectoryBuilder strafeLeft(double distance) {
        return lineTo(currentPose.pos().added(new Vector2d(
                distance * Math.cos(currentPose.heading() + Math.PI / 2),
                distance * Math.sin(currentPose.heading() + Math.PI / 2)
        )));
    }

    public TrajectoryBuilder strafeRight(double distance) {
        return strafeLeft(-distance);
    }

    public TrajectoryBuilder splineThrough(SplinePath.Type type, Pose2d... waypoints) {
        List<Pose2d> modifiedWaypoints = new ArrayList<>(Arrays.asList(waypoints));
        modifiedWaypoints.add(0, currentPose);
        CompositePath spline = CompositePath.fitSpline(type, modifiedWaypoints);
        if (composite) {
            compositeSegments.add(spline);
        } else {
            motionSegments.add(new ParametricSegment(spline));
        }
        currentPose = spline.end();
        return this;
    }

    public TrajectoryBuilder splineThrough(Pose2d... waypoints) {
        return splineThrough(SplinePath.Type.QUINTIC_HERMITIAN, waypoints);
    }

    public TrajectoryBuilder waitFor(double seconds) {
        motionSegments.add(new WaitSegment(currentPose, seconds));
        return this;
    }

    public TrajectoryBuilder beginComposite() {
        if (composite) {
            closeComposite();
        }
        composite = true;
        return this;
    }

    public TrajectoryBuilder closeComposite() {
        composite = false;
        motionSegments.add(new ParametricSegment(new CompositePath(compositeSegments)));
        compositeSegments = new ArrayList<>();
        return this;
    }

    public Trajectory build() {
        if (composite) {
            closeComposite();
        }
        return new Trajectory(motionSegments);
    }
}
