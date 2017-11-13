package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;

import java.util.Locale;

/**
 * Class representing a linear path segment
 */
public class LinearSegment implements PathSegment {
    private Vector2d start, end, seg;
    private MotionProfile profile;

    public LinearSegment(Vector2d start, Vector2d end) {
        this.start = start;
        this.end = end;
        this.seg = this.start.negated().add(this.end);
        MotionState startState = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(length(), 0);
        this.profile = MotionProfileGenerator.generateProfile(startState, goal, DriveConstants.AXIAL_CONSTRAINTS);
    }

    public Vector2d start() {
        return start;
    }

    public Vector2d end() {
        return end;
    }

    public Vector2d seg() {
        return seg;
    }

    public double length() {
        return this.seg.norm();
    }

    /** @param t range [0, 1] inclusive */
    public Vector2d get(double t) {
        return this.seg.multiplied(t).add(this.start);
    }

    /**
     * @return start if t <= 0, end if t >= 1, and {@link LinearSegment#get(double)} otherwise
     */
    public Vector2d getBounded(double t) {
        if (t <= 0) {
            return this.start.copy();
        } else if (t >= 1) {
            return this.end.copy();
        } else {
            return get(t);
        }
    }

    /** @return [0, 1] position on curve; NaN if not on curve */
    public double getPosition(Vector2d point) {
        Vector2d adj = this.start.negated().add(point);
        double tX = adj.x() / seg.x();
        double tY = adj.y() / seg.y();
        if (Math.abs(seg.x()) < Vector2d.EPSILON) {
            return tY;
        } else if (Math.abs(seg.y()) < Vector2d.EPSILON) {
            return tX;
        } else if (Math.abs(tX - tY) < Vector2d.EPSILON) {
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
        double a = seg.dot(start.negated().add(point));
        return a / seg.dot(seg);
    }

    public static double getDistance(Vector2d a, Vector2d b) {
        return a.negated().add(b).norm();
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof LinearSegment) {
            LinearSegment otherSegment = (LinearSegment) other;
            return otherSegment.start.equals(start) && otherSegment.end.equals(end);
        }
        return false;
    }

    @Override
    public String toString() {
        return seg.toString();
    }

    public String getEquation() {
        return String.format(Locale.ENGLISH, "<%f + %ft, %f + %ft>", start.x(), seg.x(), start.y(), seg.y());
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d getPose(double time) {
        return new Pose2d(new Vector2d(profile.get(time).x + start.x(), start.y()), 0);
    }

    @Override
    public Pose2d getPoseVelocity(double time) {
        return new Pose2d(new Vector2d(profile.get(time).v, 0), 0);
    }

    @Override
    public Pose2d getPoseAcceleration(double time) {
        return new Pose2d(new Vector2d(profile.get(time).a, 0), 0);
    }
}
