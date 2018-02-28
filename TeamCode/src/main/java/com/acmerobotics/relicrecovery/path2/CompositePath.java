package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class CompositePath implements ParametricPath {
    private List<ParametricPath> segments;

    public CompositePath(List<ParametricPath> segments) {
        this.segments = segments;
    }

    public static CompositePath fitSpline(Pose2d... waypoints) {
        return fitSpline(SplinePath.Type.QUINTIC_HERMITIAN, waypoints);
    }

    public static CompositePath fitSpline(SplinePath.Type type, Pose2d... waypoints) {
        if (waypoints.length < 2) {
            throw new IllegalArgumentException("2 waypoints are required to fit a spline");
        }
        List<ParametricPath> segments = new ArrayList<>();
        for (int i = 0; i < waypoints.length - 1; i++) {
            segments.add(new SplinePath(type, waypoints[i], waypoints[i + 1]));
        }
        return new CompositePath(segments);
    }

    private ParametricPath finalSegment() {
        return segments.get(segments.size() - 1);
    }

    @Override
    public double length() {
        double length = 0;
        for (ParametricPath segment : segments) {
            length += segment.length();
        }
        return length;
    }

    @Override
    public Pose2d start() {
        return segments.get(0).start();
    }

    @Override
    public Pose2d end() {
        return finalSegment().end();
    }

    @Override
    public Pose2d getPose(double displacement) {
        if (displacement < 0) {
            return start();
        }

        for (ParametricPath segment : segments) {
            if (displacement <= segment.length()) {
                return segment.getPose(displacement);
            } else {
                displacement -= segment.length();
            }
        }

        return end();
    }

    @Override
    public Pose2d getDerivative(double displacement) {
        if (displacement < 0) {
            return segments.get(0).getDerivative(0);
        }

        for (ParametricPath segment : segments) {
            if (displacement <= segment.length()) {
                return segment.getDerivative(displacement);
            } else {
                displacement -= segment.length();
            }
        }

        return finalSegment().getDerivative(finalSegment().length());
    }

    @Override
    public Pose2d getSecondDerivative(double displacement) {
        if (displacement < 0) {
            return segments.get(0).getSecondDerivative(0);
        }

        for (ParametricPath segment : segments) {
            if (displacement <= segment.length()) {
                return segment.getSecondDerivative(displacement);
            } else {
                displacement -= segment.length();
            }
        }

        return finalSegment().getSecondDerivative(finalSegment().length());
    }
}
