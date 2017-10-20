package com.acmerobotics.relicrecovery.motion;

/**
 * Created by kelly on 10/16/2017.
 *
 */

public class MotionGoal {

    public double pos;
    public double maxAbsV;

    public MotionGoal (double pos, double maxAbsV) {
        this.pos = pos;
        this.maxAbsV = maxAbsV;
    }

    public MotionGoal flipped() {
        return new MotionGoal(-pos, maxAbsV);
    }

}
