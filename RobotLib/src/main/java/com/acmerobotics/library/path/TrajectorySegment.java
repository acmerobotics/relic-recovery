package com.acmerobotics.library.path;

import com.acmerobotics.library.localization.Pose2d;

public interface TrajectorySegment {
    double duration();
    Pose2d start();
    Pose2d end();
    Pose2d getPose(double time);
    Pose2d getVelocity(double time);
    Pose2d getAcceleration(double time);
    void stopPrematurely(double time);
}
