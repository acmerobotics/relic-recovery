package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;

import java.util.Locale;

/**
 * Class representing a single segment of a piecewise linear path
 */
public class LinearSegment implements PathSegment {
    private Pose2d start, end;
    private Vector2d seg;

    public LinearSegment(Pose2d start, Pose2d end) {
        this.start = start;
        this.end = end;
        this.seg = this.start.pos().negated().add(this.end.pos());
    }

    public Pose2d start() {
        return start;
    }

    public Pose2d end() {
        return end;
    }

    public Vector2d seg() {
        return seg;
    }

    public double length() {
        return this.seg.norm();
    }

    /** @param t range [0, 1] inclusive */
    public Pose2d getPose(double t) {
        Vector2d interpolatedPos = this.seg.multiplied(t).add(this.start.pos());
        double interpolatedHeading = this.start.heading() + t * (this.start.heading() - this.end.heading());
        return new Pose2d(interpolatedPos, interpolatedHeading);
    }

    /**
     * @return start if t <= 0, end if t >= 1, and {@link LinearSegment#getPose(double)} otherwise
     */
    public Pose2d getBoundedPose(double t) {
        if (t <= 0) {
            return this.start.copy();
        } else if (t >= 1) {
            return this.end.copy();
        } else {
            return getPose(t);
        }
    }

    /** @return [0, 1] position on curve; NaN if not on curve */
    public double getPosition(Vector2d point) {
        Vector2d adj = this.start.pos().negated().add(point);
        double tX = adj.x() / seg.x();
        double tY = adj.y() / seg.y();
        if (Math.abs(seg.x()) < Path.EPSILON) {
            return tY;
        } else if (Math.abs(seg.y()) < Path.EPSILON) {
            return tX;
        } else if (Math.abs(tX - tY) < Path.EPSILON) {
            return (tX + tY) / 2.0;
        } else {
            return Double.NaN;
        }
    }

    public boolean contains(Vector2d point) {
        double pos = getPosition(point);
        return !Double.isNaN(pos) && pos >= 0 && pos <= 1;
    }

    public double getClosestPositionOnPath(Vector2d point) {
        double a = seg.dot(start.pos().negated().add(point));
        return a / seg.dot(seg);
    }

    public static double getDistance(Vector2d a, Vector2d b) {
        return a.negated().add(b).norm();
    }

    public boolean equals(LinearSegment s) {
        return s.start.equals(start) && s.end.equals(end);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "<%f, %f> to <%f, %f>", start.x(), start.y(), end.x(), end.y());
    }

    public String getEquation() {
        return String.format(Locale.ENGLISH, "<%f + %ft, %f + %ft>", start.x(), seg.x(), start.y(), seg.y());
    }

    @Override
    public Pose2d getPoseUpdate(Pose2d currentPose) {
        return null;
    }
}
