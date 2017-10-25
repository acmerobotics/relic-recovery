package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionProfile;

/**
 * @author kellyrm
 */

public abstract class PathSegment {

    protected Pose2d start;
    protected Pose2d end;
    protected MotionProfile posProfile;
    protected MotionProfile headingProfile;

}
