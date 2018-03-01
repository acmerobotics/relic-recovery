package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionProfile;

public class WaitSegment implements PathMotionSegment {
    private Pose2d pose;
    private double duration;

    public WaitSegment(Pose2d pose, double duration) {
        this.pose = pose;
        this.duration = duration;
    }

    @Override
    public PathMotionProfile profile() {
        return new WaitSegmentMotionProfile(pose, duration);
    }

    @Override
    public PathMotionProfile stoppingProfile(PathMotionProfile profile, double time) {
        return new WaitSegmentMotionProfile(pose, 0);
    }

    private class WaitSegmentMotionProfile implements PathMotionProfile {
        private Pose2d pose;
        private double duration;

        public WaitSegmentMotionProfile(Pose2d pose, double duration) {
            this.pose = pose;
            this.duration = duration;
        }

        @Override
        public double duration() {
            return duration;
        }

        @Override
        public Pose2d start() {
            return pose;
        }

        @Override
        public Pose2d end() {
            return pose;
        }

        @Override
        public Pose2d getPose(double time) {
            return pose;
        }

        @Override
        public Pose2d getVelocity(double time) {
            return new Pose2d(0, 0, 0);
        }

        @Override
        public Pose2d getAcceleration(double time) {
            return new Pose2d(0, 0, 0);
        }

        @Override
        public MotionProfile rawProfile() {
            throw new UnsupportedOperationException();
        }
    }
}
