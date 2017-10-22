package com.acmerobotics.relicrecovery.util;

/**
 * @author kellyrm
 */

public class MathUtil {

    //Static class
    private MathUtil() {}

    /**
     * f(x) = ax^3 + bx^2 + cx + d
     * @return roots of f
     */
    public static double[] solveCubic(double a, double b, double c, double d) {
        double[] result;
        if (a != 1) {
            b = b / a;
            c = c / a;
            d = d / a;
        }

        double p = c / 3 - b * b / 9;
        double q = b * b * b / 27 - b * c / 6 + d / 2;
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
            result[i] = (result[i] - b / 3);
        }
        return result;
    }

    /**
     * f(x) = ax^2 + bx + c
     * @return roots of f
     */
    public static double[] solveQuadratic(double a, double b, double c) {
        double dis = Math.pow(b, 2) - (4.0 * a * c);
        if (dis < 0) {
            return new double[]{}; // no solutions
        }
        if (dis == 0) {
            double s = -b / (2.0 * a);
            return new double[]{s}; //one solution
        }
        dis = Math.sqrt(dis);
        double s1 = (-b + dis) / (2.0 * a);
        double s2 = (-b - dis) / (2.0 * a);
        return new double[]{s1, s2};
    }
}
