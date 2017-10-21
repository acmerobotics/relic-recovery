package com.acmerobotics.relicrecovery.motion;

/**
 * @author kellyrm
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

    /**
     * Extrapolate the initial conditions to get a motion state at a given time
     * @param t time
     * @return the motion state at time t
     */
    public MotionState get(double t) {
        t = Math.max(Math.min(start.t + dt, t), start.t);
        t -= start.t;
        double t2 = t * t;
        double t3 = t2 * t;
        double a = start.j * t + start.a;
        double v = .5 * start.j * t2 + start.a * t + start.v;
        double x = (1.0/6.0) * start.j * t3 + .5 * start.a * t2 + start.v * t + start.x;
        return new MotionState (x, v, a, start.j, start.t + dt);
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


}
