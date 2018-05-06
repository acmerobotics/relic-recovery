package com.acmerobotics.library.motion;

/**
 * motion segment with constant jerk
 */

public class MotionSegment {

    private MotionState start;
    private double dt;

    /**
     * constructs a motion profile with constant jerk over an interval
     * @param start motion state at the beginning of the segment
     * @param dt duration of the motion segment
     */
    public MotionSegment(MotionState start, double dt) {
        this.start = start;
        this.dt = dt;
    }

    public MotionState get(double t) {
        return start.get(t);
    }

    /**
     * get the start state
     * @return the start state
     */
    public MotionState start() {
        return start;
    }

    /**
     * get the end state of the segment
     * @return the end state
     */
    public MotionState end() {
        return get(start.t + dt);
    }

    /**
     * get the duration of the segment
     * @return the duration
     */
    public double dt() {
        return dt;
    }

    /**
     * get the first time, within the bounds of the segment, that it will be at a position
     * refer to MotionSegment#timesAtPos
     * @param pos the position
     * @return -1 if it never passes through the position within the time bounds or if the segment is always at pos
     */
    public double timeAtPos(double pos) {
        for (double time: start.timesAtPos(pos)) {
            if (containsTime(time)) {
                return time;
            }
        }
        return -1.0;
    }

    /**
     * get the first time, within the bounds of the segment, that velocity will be at vel
     * @param vel the velocity to check for
     * @return -1 if it never reaches vel or is always at vel
     */
    public double timeAtVel(double vel) {
        for (double time: start.timesAtVel(vel)) {
            if (containsTime(time)) {
                return time;
            }
        }
        return -1.0;
    }

    /**
     * check if the segment contains a particular time
     * @param t time
     * @return true if it contains the time
     */
    public boolean containsTime(double t) {
        return start().t <= t && t <= end().t;
    }

    public MotionState getByPos(double pos) {
        return get(Math.min(end().t, Math.max(0, timeAtPos(pos))));
    }

}
