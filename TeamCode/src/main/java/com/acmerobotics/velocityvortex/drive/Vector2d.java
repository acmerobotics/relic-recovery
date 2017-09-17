package com.acmerobotics.velocityvortex.drive;

public class Vector2d {

    public static final double EPSILON = 0.00001;

    private double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d copy() {
        return new Vector2d(x, y);
    }

    public Vector2d normalize() {
        double norm = norm();
        if (norm < EPSILON) {
            x = 1;
            y = 0;
        } else {
            multiply(1.0 / norm());
        }
        return this;
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double dot(Vector2d other) {
        return x * other.x() + y * other.y();
    }

    public Vector2d multiply(double scalar) {
        this.x *= scalar;
        this.y *= scalar;
        return this;
    }

    public Vector2d multiplied(double scalar) {
        return copy().multiply(scalar);
    }

    public Vector2d add(Vector2d other) {
        this.x += other.x();
        this.y += other.y();
        return this;
    }

    public Vector2d added(Vector2d other) {
        return copy().add(other);
    }

    public Vector2d negate() {
        x = -x;
        y = -y;
        return this;
    }

    public Vector2d negated() {
        return copy().negate();
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public boolean equals(Vector2d other) {
        return Math.abs(x - other.x) < EPSILON && Math.abs(y - other.y) < EPSILON;
    }

    @Override
    public String toString() {
        return "<" + x + ", " + y + ">";
    }
}
