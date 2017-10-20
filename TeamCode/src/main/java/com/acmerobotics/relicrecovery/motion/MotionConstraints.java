package com.acmerobotics.relicrecovery.motion;

/**
 * Created by kelly on 10/14/2017.
 *
 */

public class MotionConstraints {

    public double maxV, maxA, maxJ;

    public double posTolerance, velTolerance;

    public enum END_BEHAVIOR {
        OVERSHOOT,
        VIOLATE_MAX_ABS_V,
        VIOLATE_MAX_ABS_A
    }

    public END_BEHAVIOR endBehavior;
}
