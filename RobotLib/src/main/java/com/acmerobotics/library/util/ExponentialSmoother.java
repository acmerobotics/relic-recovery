package com.acmerobotics.library.util;

/**
 * This class implements a simple smoother based on an exponential moving average.
 */

public class ExponentialSmoother {

    private double smoothingFactor, average;
    private boolean hasUpdated;

    public ExponentialSmoother(double smoothingFactor) {
        this.smoothingFactor = smoothingFactor;
    }

    /**
     * Update the smoother with a new data value
     *
     * @param newValue new data value
     * @return the smoothed value
     */
    public double update(double newValue) {
        if (!hasUpdated) {
            average = newValue;
            hasUpdated = true;
        } else {
            average = smoothingFactor * newValue + (1 - smoothingFactor) * average;
        }
        return average;
    }

    public void reset() {
        hasUpdated = false;
    }

    public void setSmoothingFactor(double smoothingFactor) {
        this.smoothingFactor = smoothingFactor;
    }

}
