package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

import java.util.ArrayList;
import java.util.List;

public class PathBuilder {
    private Pose2d currentPose;
    private List<PathSegment> segments;

    public PathBuilder(Pose2d pose) {
        currentPose = pose;
        segments = new ArrayList<>();
    }

    public PathBuilder lineTo(Vector2d pos) {
        Pose2d pose = new Pose2d(pos, currentPose.heading());
        segments.add(new LineSegment(currentPose, pose));
        currentPose = pose;
        return this;
    }

    public PathBuilder turn(double angle) {
        segments.add(new PointTurn(currentPose, angle));
        currentPose = currentPose.added(new Pose2d(0, 0, angle));
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

    public PathBuilder waitFor(double seconds) {
        segments.add(new WaitSegment(currentPose, seconds));
        return this;
    }

    public Path build() {
        return new Path(segments);
    }
}
