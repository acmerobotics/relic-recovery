package com.acmerobotics.relicrecovery.motion;

/**
 * @author kellyrm
 * constraints for a motion profile
 */

public class MotionConstraints {

    /**
     * max absolute value of velocity
     */
    public double maxV;

    /**
     * max absolute value of acceleration
     */
    public double maxA;

    /**
     * max absolute value of jerk
     */
    public double maxJ;

    /**
     * behavior in the case that the profile cannot finish slowing before it reaches the target position
     */
    public enum EndBehavior {
        /**
         * Go past the goal and then come back to it - useful if maxA and maxV really can't be violated
         */
        OVERSHOOT,

        /**
         * Ignore the target velocity but still slow down as much as possible - useful if there is going to be another move after this one
         */
        VIOLATE_MAX_ABS_V,

        /**
         * Slam on the brakes and stop in time, useful if you don't want to run into something
         * As a side effect maxJ is also violated, it is not going to be pretty no matter what
         */
        VIOLATE_MAX_ABS_A
    }

    public EndBehavior endBehavior;

    public MotionConstraints(double maxV, double maxA, double maxJ, EndBehavior endBehavior) {
        this.maxV = maxV;
        this.maxA = maxA;
        this.maxJ = maxJ;
        this.endBehavior = endBehavior;
    }

}
