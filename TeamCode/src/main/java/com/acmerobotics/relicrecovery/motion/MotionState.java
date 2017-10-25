package com.acmerobotics.relicrecovery.motion;

import com.acmerobotics.relicrecovery.util.MathUtil;

/**
 *  @author kellyrm
 *  fully defined state of one dimensional motion
 */

public class MotionState {

    /**
     * position
     */
    public double x;

    /**
     * velocity - x/t
     */
    public double v;

    /**
     * acceleration - x/t^2
     */
    public double a;

    /**
     * jerk - x/t^3
     */
    public double j;

    /**
     * time
     */
    public double t;


    public MotionState (double x, double v, double a, double j, double t) {
        this.x = x;
        this.v = v;
        this.a = a;
        this.j = j;
        this.t = t;
    }

    /**
     * Extrapolate the initial conditions to get a motion state at a given time
     * x(t) = (1/6)jt^3 + (1/2)a(0)t^2 + v(0)t + x(0)
     * v(t) = (1/2)jt^2 + a(0)t + v(0)
     * a(t) = jt + a(0)
     * @param time time
     * @return the motion state at time t
     */
    public MotionState get(double time) {
        time -= this.t;
        double t2 = time * time;
        double t3 = t2 * time;
        double a = this.j * time + this.a;
        double v = .5 * this.j * t2 + this.a * time + this.v;
        double x = (1.0/6.0) * this.j * t3 + .5 * this.a * t2 + this.v * time + this.x;
        return new MotionState (x, v, a, this.j, time + this.t);
    }

    /**
     * flip the motion state
     * @return a new motion state with everything negated
     */
    public MotionState flipped() {
        return new MotionState(-x, -v, -a, -j, t);
    }


    /**
     * get all times that the current state will reach a position
     * @param pos the position to check for
     * @return the times it will be at pos - returns {} if it will never be at pos or if it is always at pos
     */
    public double[] timesAtPos(double pos) {
        //third degree case
        MotionState stateAt0 = get(0);
        if (j != 0) {
            return MathUtil.solveCubic(stateAt0.j / 6.0, stateAt0.a / 2.0, stateAt0.v, stateAt0.x - pos);
        }
        //second degree case
        if (this.a != 0) {
            return MathUtil.solveQuadratic(stateAt0.a/2.0, stateAt0.v, stateAt0.x - pos);
        }
        //first degree case
        if (this.v != 0) {
            return new double [] {(pos - stateAt0.x) / stateAt0.v };
        }
        return new double[] {}; //we are either always or never at pos
    }

    /**
     * find all times the state will be at vel
     * @param vel velocity to check
     * @return {} if it never reaches vel or if it is always at vel, otherwise the times it is at vel
     */
    public double[] timesAtVel(double vel) {
        MotionState stateAt0 = get(0);
        //second degree case
        if (this.j != 0) {
            return MathUtil.solveQuadratic(stateAt0.j / 2.0, stateAt0.a, stateAt0.v - vel);
        }
        //first degree case
        if (this.a != 0) {
            return new double [] {(vel - stateAt0.v) / stateAt0.a};
        }
        return new double[] {};
    }


}
