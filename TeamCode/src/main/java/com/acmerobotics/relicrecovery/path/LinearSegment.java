package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;

/**
 * Class representing a linear path segment
 */
public class LinearSegment implements PathSegment {
    private Pose2d start;
    private Vector2d end, seg;
    private MotionProfile profile;

    public LinearSegment(Pose2d start, Vector2d end) {
        this.start = start;
        this.end = end;
        this.seg = end.added(start.pos().negated());
        MotionState startState = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(length(), 0);
        this.profile = MotionProfileGenerator.generateProfile(startState, goal, DriveConstants.AXIAL_CONSTRAINTS);
    }

    public Vector2d start() {
        return start.pos();
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
        return start + " to " + end;
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
