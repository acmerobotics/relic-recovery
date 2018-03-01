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

    public ParametricMotionSegment(ParametricPath path) {
        this.path = path;
    }

    @Override
    public PathMotionProfile profile() {
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(path.length(), 0);
        return new ParametricPathMotionProfile(start, goal);
    }

    @Override
    public PathMotionProfile stoppingProfile(PathMotionProfile profile, double time) {
        MotionProfile rawProfile = profile.rawProfile();
        MotionState start = rawProfile.get(time);
        double remainingDistance = rawProfile.end().x - start.x;
        double decelerationDistance = rawProfile.end().x - rawProfile.segments().get(-3).start().x;
        double goalPosition;
        if (remainingDistance > decelerationDistance) {
            goalPosition = start.x + decelerationDistance;
        } else {
            goalPosition = path.length();
        }
        MotionGoal goal = new MotionGoal(goalPosition, 0);
        return new ParametricPathMotionProfile(start, goal);
    }

    private class ParametricPathMotionProfile implements PathMotionProfile {
        private MotionProfile profile;

        public ParametricPathMotionProfile(MotionState start, MotionGoal goal) {
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
        public MotionProfile rawProfile() {
            return profile;
        }
    }
}
