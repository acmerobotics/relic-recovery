package com.acmerobotics.relicrecovery.motion;

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
        if (j != 0) {
            //according to the internet this is a way to find roots of cubic functions, and it appears to work
            //don't ask be what is going on
            //i'm not quite sure why there are trig functions involved
            double a = this.a / 2.0;
            double b = v;
            double c = x - pos;
            double d = j / 6.0;
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
                result[i] = this.t + (result[i] - a / 3); //gotta do this because above it assumes that the state is at 0
            }
            return result;
        }
        //second degree case
        if (this.a != 0) {
            //this ones easy, just solve the quadratic formula
            double a = this.a / 2.0;
            double b = this.v;
            double c = this.x - pos;
            double dis = Math.pow(b, 2) - (4.0 * a * c);
            if (dis < 0) {
                return new double[]{}; //never at pos
            }
            if (dis == 0) {
                double t = -b / (2.0 * a);
                return new double[]{t + this.t}; //only at pos once
            }
            dis = Math.sqrt(dis);
            double t1 = (-b + dis) / (2.0 * a);
            double t2 = (-b - dis) / (2.0 * a);
            return new double[]{t1 + this.t, t2 + this.t};
        }
        //first degree case
        if (this.v != 0) {
            double t = (pos - this.x) / this.t;
            return new double [] {t + this.t};
        }
        return new double[] {}; //we are either always or never at pos
    }

    /**
     * find all times the state will be at vel
     * @param vel velocity to check
     * @return {} if it never reaches vel or if it is always at vel, otherwise the times it is at vel
     */
    public double[] timesAtVel(double vel) {
        //second degree case
        if (this.j != 0) {
            double a = this.j / 2.0;
            double b = this.a;
            double c = this.v - vel;
            double dis = Math.pow(b, 2) - (4.0 * a * c);
            if (dis < 0) {
                return new double[]{}; //never at vel
            }
            if (dis == 0) {
                double t = -b / (2.0 * a);
                return new double[]{t + this.t}; //only at vel once
            }
            dis = Math.sqrt(dis);
            double t1 = (-b + dis) / (2.0 * a);
            double t2 = (-b - dis) / (2.0 * a);
            return new double[]{t1 + this.t, t2 + this.t};
        }
        //first degree case
        if (this.a != 0) {
            double t = (vel - this.v) / this.a;
            return new double [] {t + this.t};
        }
        return new double[] {};
    }


}
