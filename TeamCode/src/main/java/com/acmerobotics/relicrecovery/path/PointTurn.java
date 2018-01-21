package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

import java.util.Locale;

/**
 * Created by ryanbrott on 11/5/17.
 */

public class PointTurn implements PathSegment {
    private Pose2d initialPose;
    private double angle;
    private MotionProfile profile;

    public PointTurn(Pose2d initialPose, double angle) {
        this.initialPose = initialPose;
        this.angle = angle;
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(angle, 0);
        this.profile = MotionProfileGenerator.generateProfile(start, goal, MecanumDrive.POINT_TURN_CONSTRAINTS);
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof PointTurn) {
            PointTurn otherTurn = (PointTurn) other;
            return Math.abs(angle - otherTurn.angle) < Vector2d.EPSILON;
        }
        return false;
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "PointTurn[%s, %1.4f (%.2f deg)]", initialPose.toString(), angle, Math.toDegrees(angle));
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d getPose(double time) {
        return new Pose2d(initialPose.pos(), Angle.norm(profile.get(time).x + initialPose.heading()));
    }

    @Override
    public Pose2d getPoseVelocity(double time) {
        return new Pose2d(new Vector2d(0, 0), profile.get(time).v);
    }

    @Override
    public Pose2d getPoseAcceleration(double time) {
        return new Pose2d(new Vector2d(0, 0), profile.get(time).a);
    }
}
