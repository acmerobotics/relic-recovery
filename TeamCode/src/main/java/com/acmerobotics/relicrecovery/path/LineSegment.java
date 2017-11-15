package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;

import java.util.Locale;

/**
 * Class representing a linear path segment
 */
public class LineSegment implements PathSegment {
    private Pose2d start, end;
    private Vector2d seg;
    private MotionProfile profile;

    public LineSegment(Pose2d start, Pose2d end) {
        this.start = start;
        this.end = end;
        this.seg = end.pos().added(start.pos().negated());
        double lineHeading = Math.atan2(seg.y(), seg.x());
        MotionGoal goal;
        if (Math.abs(Angle.norm(lineHeading - start.heading())) < Vector2d.EPSILON) {
            goal = new MotionGoal(length(), 0);
        } else {
            goal = new MotionGoal(-length(), 0);
        }
        MotionState startState = new MotionState(0, 0, 0, 0, 0);
        this.profile = MotionProfileGenerator.generateProfile(startState, goal, DriveConstants.AXIAL_CONSTRAINTS);
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

    @Override
    public boolean equals(Object other) {
        if (other instanceof LineSegment) {
            LineSegment otherSegment = (LineSegment) other;
            return otherSegment.start.equals(start) && otherSegment.end.equals(end);
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "LineSegment[%s to %s]", start.toString(), end.toString());
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d getPose(double time) {
        double distance = profile.get(time).x;
        Pose2d relativePose = new Pose2d(distance * Math.cos(start.heading()), distance * Math.sin(start.heading()));
        return relativePose.added(start);
    }

    @Override
    public Pose2d getPoseVelocity(double time) {
        double velocity = profile.get(time).v;
        return new Pose2d(velocity * Math.cos(start.heading()), velocity * Math.sin(start.heading()));
    }

    @Override
    public Pose2d getPoseAcceleration(double time) {
        double acceleration = profile.get(time).a;
        return new Pose2d(acceleration * Math.cos(start.heading()), acceleration * Math.sin(start.heading()));
    }
}
