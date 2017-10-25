package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.MotionState;

/**
 * @author kellyrm
 *
 */

public class LinearPathSegment extends PathSegment{

    public LinearPathSegment (Pose2d start, Pose2d end, MotionConstraints posConstraints, MotionConstraints headingConstraints, MotionState startState) {
        this.start = start;
        this.end = end;


    }

}
