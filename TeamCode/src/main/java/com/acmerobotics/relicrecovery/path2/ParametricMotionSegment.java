package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
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
        return path.getVelocity(motionState.x).multiplied(motionState.v);
    }

    @Override
    public Pose2d getAcceleration(double time) {
        MotionState motionState = profile.get(time);
        return path.getAcceleration(motionState.x).multiplied(Math.pow(motionState.x, 2))
                .added(path.getVelocity(motionState.x).multiplied(motionState.a));
    }
}
