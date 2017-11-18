package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

/**
 * Class representing a segment of a {@link Path}
 */

public interface PathSegment {
    /** @return duration in seconds */
    double duration();

    /** @return the pose at a time t in seconds */
    Pose2d getPose(double time);

    /** @return the pose velocity at time t in seconds */
    Pose2d getPoseVelocity(double time);

    /** @return the pose acceleration at time t in seconds */
    Pose2d getPoseAcceleration(double time);
}
