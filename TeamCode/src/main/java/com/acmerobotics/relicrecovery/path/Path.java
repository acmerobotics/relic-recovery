package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing a path composed of multiple {@link PathSegment}s
 */
public class Path {

    private List<PathSegment> segments;
    private double duration;

    public Path(List<PathSegment> segments) {
        this.segments = new ArrayList<>();
        for (PathSegment segment : segments) {
            addSegment(segment);
        }
    }

    public static Path createFromPoses(List<Pose2d> poses) {
        List<PathSegment> segments = new ArrayList<>();
        double heading = poses.get(0).heading();
        for (int i = 1; i < poses.size(); i++) {
            Pose2d lastPose = poses.get(i - 1);
            Pose2d pose = poses.get(i);
            double deltaX = pose.x() - lastPose.x();
            double deltaY = pose.y() - lastPose.y();
            double newHeading = Angle.norm(Math.atan2(deltaY, deltaX));
            double turnAngle = Angle.norm(newHeading - heading);
            if ((Math.abs(deltaX) > Vector2d.EPSILON || Math.abs(deltaY) > Vector2d.EPSILON) &&
                Math.abs(turnAngle) > Vector2d.EPSILON) {
                segments.add(new PointTurn(lastPose.pos(), turnAngle));
                heading = newHeading;
            }
            double length = lastPose.pos().negated().add(pose.pos()).norm();
            if (length > Vector2d.EPSILON) {
                segments.add(new LinearSegment(lastPose.pos(), pose.pos()));
            }
        }
        Pose2d finalPose = poses.get(poses.size() - 1);
        double finalTurnAngle = Angle.norm(finalPose.heading() - heading);
        if (Math.abs(finalTurnAngle) > Vector2d.EPSILON) {
            segments.add(new PointTurn(finalPose.pos(), finalTurnAngle));
        }
        return new Path(segments);
    }

    @Override
    public String toString() {
        StringBuilder builder = new StringBuilder();
        for (PathSegment s : segments) {
            builder.append(s.toString());
            builder.append("\n");
        }
        return builder.toString();
    }

    public PathSegment getSegment(int i) {
        return segments.get(i);
    }

    public List<PathSegment> getSegments() {
        return segments;
    }

    public void addSegment(PathSegment segment) {
        this.segments.add(segment);
        duration += segment.duration();
    }

    public void removeSegment(PathSegment segment) {
        this.segments.remove(segment);
        duration -= segment.duration();
    }

    public int size() {
        return segments.size();
    }

    public double duration() {
        return duration;
    }

    public Pose2d getPose(double time) {
        for (PathSegment segment : segments) {
            if (time <= segment.duration()) {
                return segment.getPose(time);
            }
            time -= segment.duration();
        }
        return null;
    }

    public Pose2d getPoseVelocity(double time) {
        for (PathSegment segment : segments) {
            if (time <= segment.duration()) {
                return segment.getPoseVelocity(time);
            }
            time -= segment.duration();
        }
        return null;
    }

    public Pose2d getPoseAcceleration(double time) {
        for (PathSegment segment : segments) {
            if (time <= segment.duration()) {
                return segment.getPoseAcceleration(time);
            }
            time -= segment.duration();
        }
        return null;
    }
}
