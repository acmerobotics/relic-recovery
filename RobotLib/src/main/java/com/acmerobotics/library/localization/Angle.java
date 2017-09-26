package com.acmerobotics.library.localization;

/**
 * Created by kelly on 9/21/2017.
 *
 */

public class Angle {

    private double value;

    public Angle() {
        value = 0;
    }

    public Angle(double value) {
        this.value = value;
        norm();
    }

    public Angle add(Angle other) {
        return new Angle(this.value + other.value);
    }

    public Angle subtract(Angle other) {
        return new Angle(this.value - other.value);
    }

    private void norm() {
        while (Math.abs(value) > Math.PI) {
            value -= 2 * Math.PI * Math.signum(value);
        }
    }

    public double value() {
        return value;
    }

}
