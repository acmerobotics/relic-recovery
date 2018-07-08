package com.acmerobotics.relicrecovery;

import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.trajectory.DriveConstraints;

public class RelicRecoveryConstraints extends DriveConstraints {
    public RelicRecoveryConstraints(double maximumVelocity, double maximumAcceleration, double maximumAngularVelocity, double maximumAngularAcceleration, double maximumCentripetalAcceleration) {
        super(maximumVelocity, maximumAcceleration, maximumAngularVelocity, maximumAngularAcceleration, maximumCentripetalAcceleration);
    }

    @Override
    public double maximumVelocity(Pose2d pose, Pose2d poseDeriv, Pose2d poseSecondDeriv) {
        double maxVel = getMaximumVelocity();
        if (Math.abs(pose.y()) < 28 && Math.signum(pose.y()) != Math.signum(poseDeriv.y())) {
            maxVel = 10.0;
        }

        return Math.min(maxVel, super.maximumVelocity(pose, poseDeriv, poseSecondDeriv));
    }
}
