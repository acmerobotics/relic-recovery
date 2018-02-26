package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;

public interface ParametricPath {
    double length();
    Pose2d start();
    Pose2d end();
    Pose2d getPose(double displacement);
    Pose2d getVelocity(double displacement);
    Pose2d getAcceleration(double displacement);
}
