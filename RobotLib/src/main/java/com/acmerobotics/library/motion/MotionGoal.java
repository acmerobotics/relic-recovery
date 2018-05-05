package com.acmerobotics.library.motion;

/**
 * goal of a motion profile
 */

public class MotionGoal {

    public double pos;
    /**
     * max absolute value at pos
     */
    public double maxAbsV;

    public MotionGoal (double pos, double maxAbsV) {
        this.pos = pos;
        this.maxAbsV = maxAbsV;
    }

    public MotionGoal flipped() {
        return new MotionGoal(-pos, maxAbsV);
    }

}
