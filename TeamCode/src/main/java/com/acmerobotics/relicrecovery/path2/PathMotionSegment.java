package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;

public interface PathMotionSegment {
    double duration();
    Pose2d start();
    Pose2d end();
    Pose2d getPose(double time);
    Pose2d getVelocity(double time);
    Pose2d getAcceleration(double time);
}
