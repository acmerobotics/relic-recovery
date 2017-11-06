package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;

/**
 * Created by ryanbrott on 11/5/17.
 */

public interface PathSegment {
    Pose2d getPoseUpdate(Pose2d currentPose);
}
