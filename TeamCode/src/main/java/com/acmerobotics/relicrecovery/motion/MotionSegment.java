package com.acmerobotics.relicrecovery.motion;

/**
 * Created by kelly on 10/12/2017.
 * motion segment with constant jerk
 */

public class MotionSegment {

    private MotionState start;
    private double dt;

    public MotionSegment(MotionState start, double dt) {
        this.start = start;
        this.dt = dt;
    }

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

    public MotionState start() {
        return start;
    }

    public MotionState end() {
        return get(start.t + dt);
    }

    public double dt() {
        return dt;
    }

    public double timeAtPos(double pos) {
        for (double time: timesAtPos(pos)) {
            if (containsTime(time)) {
                return time;
            }
        }
        return -1.0;
    }

    public double[] timesAtPos(double pos) {
        if (start.j != 0) {
            double a = start.a / 2.0;
            double b = start.v;
            double c = start.x - pos;
            double d = start.j / 6.0;
            double[] result;
            if (d != 1) {
                a = a / d;
                b = b / d;
                c = c / d;
            }

            double p = b / 3 - a * a / 9;
            double q = a * a * a / 27 - a * b / 6 + c / 2;
            double D = p * p * p + q * q;

            if (Double.compare(D, 0) >= 0) {
                if (Double.compare(D, 0) == 0) {
                    double r = Math.cbrt(-q);
                    result = new double[2];
                    result[0] = 2 * r;
                    result[1] = -r;
                } else {
                    double r = Math.cbrt(-q + Math.sqrt(D));
                    double s = Math.cbrt(-q - Math.sqrt(D));
                    result = new double[1];
                    result[0] = r + s;
                }
            } else {
                double ang = Math.acos(-q / Math.sqrt(-p * p * p));
                double r = 2 * Math.sqrt(-p);
                result = new double[3];
                for (int k = -1; k <= 1; k++) {
                    double theta = (ang - 2 * Math.PI * k) / 3;
                    result[k + 1] = r * Math.cos(theta);
                }

            }
            for (int i = 0; i < result.length; i++) {
                result[i] = result[i] - a / 3;
            }
            return result;
        }
        if (start.a != 0) {
            double a = start.a / 2.0;
            double b = start.v;
            double c = start.x - pos;
            double dis = Math.pow(b, 2) - (4.0 * a * c);
            if (dis < 0) {
                return new double[]{}; //never at pos
            }
            if (dis == 0) {
                double t = -b / (2.0 * a);
                return new double[]{t + start.t}; //only at pos once
            }
            dis = Math.sqrt(dis);
            double t1 = (-b + dis) / (2.0 * a);
            double t2 = (-b - dis) / (2.0 * a);
            return new double[]{t1 + start.t, t2 + start.t};
        }
        if (start.v != 0) {
            double t = (pos - start.x) / start.t;
            return new double [] {t + start.t};
        }
        return new double[] {};
    }

    public double timeAtVel(double vel) {
        for (double time: timesAtVel(vel)) {
            if (containsTime(time)) {
                return time;
            }
        }
        return -1.0;
    }

    /**
     *
     * @param vel velocity to check
     * @return {} if it never reaches vel or if it is always at vel, otherwise the times it is at vel
     */
    public double[] timesAtVel(double vel) {
        if (start.j != 0) {
            double a = start.j / 2.0;
            double b = start.a;
            double c = start.v - vel;
            double dis = Math.pow(b, 2) - (4.0 * a * c);
            if (dis < 0) {
                return new double[]{}; //never at vel
            }
            if (dis == 0) {
                double t = -b / (2.0 * a);
                return new double[]{t + start.t}; //only at vel once
            }
            dis = Math.sqrt(dis);
            double t1 = (-b + dis) / (2.0 * a);
            double t2 = (-b - dis) / (2.0 * a);
            return new double[]{t1 + start.t, t2 + start.t};
        }
        if (start.a != 0) {
            double t = (vel - start.v) / start.a;
            return new double [] {t + start.t};
        }
        return new double[] {};
    }

    public boolean containsTime(double t) {
        return start().t <= t && t <= end().t;
    }


}
