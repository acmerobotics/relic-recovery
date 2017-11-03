package com.acmerobotics.relicrecovery.localization;

/**
 * Created by kelly on 9/27/2017.
 *
 */

public class Vector2d {

    private double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public void add(Vector2d other) {
        x += other.x;
        y += other.y;
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

}
