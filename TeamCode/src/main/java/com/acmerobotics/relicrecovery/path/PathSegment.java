package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionState;

/**
 * @author kellyrm
 */

public abstract class PathSegment {

    protected Pose2d start;
    protected Pose2d end;
    protected double length;

    public Pose2d start() {
        return start;
    }

    public Pose2d end() {
        return end();
    }

}
