package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.qualcomm.hardware.motors.NeveRest20Gearmotor;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

import java.util.Locale;

/**
 * Created by ryanbrott on 11/5/17.
 */

public class PointTurn implements PathSegment {
    private Vector2d center;
    private double angle;
    private MotionProfile profile;

    public PointTurn(Vector2d center, double angle) {
        this.center = center;
        this.angle = angle;
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(angle, 0);
        this.profile = MotionProfileGenerator.generateProfile(start, goal, DriveConstants.POINT_TURN_CONSTRAINTS);
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
        return String.format(Locale.ENGLISH, "%1.4f (%.2f deg)", angle, Math.toDegrees(angle));
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d getPose(double time) {
        return new Pose2d(center, profile.get(time).x);
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
