package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;

import java.util.ArrayList;
import java.util.List;

/**
 * Class representing a path composed of multiple {@link PathSegment}s
 */
public class Path {

    public static final double EPSILON = 0.0001;

    /**
     * Report describing the distance between a path and a point
     * @see Path#getDistanceReport(Vector2d)
     */
    public static class DistanceReport {
        public LinearSegment segment;
        public Vector2d queryPoint, pathPoint;
        public double distance;
    }

    private List<LinearSegment> segments;
    private double length;

    public Path(List<Pose2d> poses) {
        segments = new ArrayList<>();
        for (int i = 0; i < poses.size() - 1; i++) {
            LinearSegment s = new LinearSegment(poses.get(i), poses.get(i+1));
            segments.add(s);
            length += s.length();
        }
    }

    @Override
    public String toString() {
        String str = "";
        for (LinearSegment s : segments) {
            str += s + "\n";
        }
        return str;
    }

    public LinearSegment getSegment(int i) {
        return segments.get(i);
    }

    public LinearSegment getSegment(Vector2d point) {
        for (LinearSegment s : segments) {
            if (s.contains(point)) {
                return s;
            }
        }
        return null;
    }

    public List<LinearSegment> getSegments() {
        return segments;
    }

    public double length() {
        return length;
    }

    public double getPosition(Vector2d point) {
        double l = 0;
        for (LinearSegment s : segments) {
            if (s.contains(point)) {
                return (l + s.getPosition(point) * s.length()) / length;
            }
            l += s.length();
        }
        return Double.NaN;
    }

    public Pose2d getPose(double t) {
        double s = t * length;
        if (s <= 0) {
            return segments.get(0).start();
        }
        for (LinearSegment seg : segments) {
            double segLength = seg.length();
            if (segLength > s) {
                return seg.getPose(s / segLength);
            }
            s -= segLength;
        }
        return segments.get(segments.size() - 1).end();
    }

    public DistanceReport getDistanceReport(Vector2d point) {
        DistanceReport report = new DistanceReport();
        report.distance = Double.POSITIVE_INFINITY;
        for (LinearSegment s : segments) {
            double t = s.getClosestPositionOnPath(point);
            double distance = LinearSegment.getDistance(point, s.getBoundedPose(t).pos());
            if (distance < report.distance) {
                report.segment = s;
                report.distance = distance;
                report.queryPoint = point;
                report.pathPoint = s.getBoundedPose(t).pos();
            }
        }
        return report;
    }

    public Vector2d getLookaheadPoint(Vector2d source, double distance) {
        // solve quadratic for circle/segment intersection
        DistanceReport report = getDistanceReport(source);
        for (int i = segments.indexOf(report.segment); i < segments.size(); i++) {
            LinearSegment s = segments.get(i);

            // substitution to make the expressions simpler
            double u = s.start().x() - source.x();
            double v = s.start().y() - source.y();

            // simple quadratic coefficients
            double a = Math.pow(s.seg().x(), 2) + Math.pow(s.seg().y(), 2);
            double b = 2 * (s.seg().x() * u + s.seg().y() * v);
            double c = Math.pow(u, 2) + Math.pow(v, 2) - Math.pow(distance, 2);

            // only care about the positive solution because t must be in [0, 1]
            double disc = Math.pow(b, 2) - 4 * a * c;
            if (disc >= 0) {
                double t = (-b + Math.sqrt(disc)) / (2 * a);
                if (t >= 0 && t <= 1) {
                    return s.getBoundedPose(t).pos();
                }
            }
        }
        return segments.get(segments.size() - 1).end().pos().copy();
    }

    public int size() {
        return segments.size();
    }
}
