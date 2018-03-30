package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

public class WaitSegment implements TrajectorySegment {
    private Pose2d pose;
    private double duration;

    public WaitSegment(Pose2d pose, double duration) {
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
    public void stopPrematurely(double time) {
        duration = time;
    }
}
