package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

import java.util.Locale;

/**
 * Class representing a linear path segment. Only supports purely translational movement.
 */
public class LineSegment implements PathSegment {
    private Pose2d start, end;
    private Vector2d seg;
    private MotionProfile profile;

    public LineSegment(Pose2d start, Pose2d end) {
        this.start = start;
        this.end = end;
        this.seg = end.pos().added(start.pos().negated());
        MotionGoal goal = new MotionGoal(length(), 0);
        MotionState startState = new MotionState(0, 0, 0, 0, 0);
        this.profile = MotionProfileGenerator.generateProfile(startState, goal, MecanumDrive.AXIAL_CONSTRAINTS);
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
        return start.added(new Pose2d(seg.multiplied(distance / length()), 0));
    }

    @Override
    public Pose2d getPoseVelocity(double time) {
        double velocity = profile.get(time).v;
        return new Pose2d(seg.multiplied(velocity / length()), 0);
    }

    @Override
    public Pose2d getPoseAcceleration(double time) {
        double acceleration = profile.get(time).a;
        return new Pose2d(seg.multiplied(acceleration / length()), 0);
    }
}
