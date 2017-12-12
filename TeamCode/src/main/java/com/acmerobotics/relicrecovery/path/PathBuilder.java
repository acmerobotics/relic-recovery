package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 12/11/17.
 */

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

    public Path build() {
        return new Path(segments);
    }
}
