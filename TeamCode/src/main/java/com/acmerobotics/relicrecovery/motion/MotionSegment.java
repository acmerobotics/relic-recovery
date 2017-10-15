package com.acmerobotics.relicrecovery.motion;

/**
 * Created by kelly on 10/12/2017.
 * motion segment with constant jerk
 */

public class MotionSegment {

    private MotionState start;
    private MotionState end;

    public MotionSegment(MotionState start, MotionState end) {

    }

    public MotionSegment(MotionState start, long dt) {
        long t2 = dt * dt;
        double a = start.j * dt + start.a;
        double v = (1/2) * start.j * t2 + start.a * dt + start.v;
        double x = (1/6) * start.j * t2 * dt + (1/2) * start.a * t2 + start.v * dt + start.x;
        end = new MotionState (x, v, a, start.j, start.t + dt);
        this.start = start;
    }

    public double dx() {
        return end.x - start.x;
    }

    public double dv() {
        return end.v - start.v;
    }

    public double da() {
        return end.a - start.a;
    }



}
