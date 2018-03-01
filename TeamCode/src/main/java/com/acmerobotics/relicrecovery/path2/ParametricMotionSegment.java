package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.path2.parametric.ParametricPath;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

public class ParametricMotionSegment implements PathMotionSegment {
    private ParametricPath path;
    private MotionProfile profile;

    public ParametricMotionSegment(ParametricPath path) {
        this.path = path;
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(path.length(), 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, MecanumDrive.AXIAL_CONSTRAINTS);
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d start() {
        return path.start();
    }

    @Override
    public Pose2d end() {
        return path.end();
    }

    @Override
    public Pose2d getPose(double time) {
        return path.getPose(profile.get(time).x);
    }

    @Override
    public Pose2d getVelocity(double time) {
        MotionState motionState = profile.get(time);
        return path.getDerivative(motionState.x).multiplied(motionState.v);
    }

    @Override
    public Pose2d getAcceleration(double time) {
        MotionState motionState = profile.get(time);
        return path.getSecondDerivative(motionState.x).multiplied(Math.pow(motionState.x, 2))
                .added(path.getDerivative(motionState.x).multiplied(motionState.a));
    }

    @Override
    public void trimRemainingDistance(double time) {
        MotionState newStart = profile.get(time);
        double remainingDistance = profile.end().x - newStart.x;
        double decelerationDistance = profile.end().x - profile.segments().get(-3).start().x;
        double goalPosition;
        if (remainingDistance > decelerationDistance) {
            goalPosition = newStart.x + decelerationDistance;
        } else {
            goalPosition = path.length();
        }
        MotionGoal goal = new MotionGoal(goalPosition, 0);
        profile = MotionProfileGenerator.generateProfile(newStart, goal, MecanumDrive.AXIAL_CONSTRAINTS);
    }
}
