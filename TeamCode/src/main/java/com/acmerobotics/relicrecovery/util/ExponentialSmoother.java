package com.acmerobotics.relicrecovery.util;

/**
 * This class implements a simple smoother based on an exponential moving average.
 */

public class ExponentialSmoother {

    private double exp, avg;
    private boolean hasUpdated;

    public ExponentialSmoother(double exp) {
        this.exp = exp;
    }

    /**
     * Update the smoother with a new data value
     *
     * @param val new data value
     * @return the smoothed value
     */
    public double update(double val) {
        if (!hasUpdated) {
            avg = val;
            hasUpdated = true;
        } else {
            avg = exp * val + (1 - exp) * avg;
        }
        return avg;
    }

}
