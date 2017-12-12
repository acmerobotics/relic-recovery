package com.acmerobotics.library.localization;

public class Vector2d {

    public static final double EPSILON = 0.00001;

    private final double x, y;

    public Vector2d(double x, double y) {
        this.x = x;
        this.y = y;
    }

    public Vector2d copy() {
        return new Vector2d(x, y);
    }

    public Vector2d normalized() {
        double norm = norm();
        if (norm < EPSILON) {
            return new Vector2d(1, 0);
        } else {
            return multiplied(1.0 / norm());
        }
    }

    public double norm() {
        return Math.hypot(x, y);
    }

    public double dot(Vector2d other) {
        return x * other.x() + y * other.y();
    }

    public Vector2d multiplied(double scalar) {
        return new Vector2d(scalar * x, scalar * y);
    }

    public Vector2d added(Vector2d other) {
        return new Vector2d(x + other.x, y + other.y);
    }

    public Vector2d negated() {
        return new Vector2d(-x, -y);
    }

    public Vector2d rotated(double angle) {
        double newX = x * Math.cos(angle) - y * Math.sin(angle);
        double newY = x * Math.sin(angle) + y * Math.cos(angle);
        return new Vector2d(newX, newY);
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Vector2d) {
            Vector2d otherVector = (Vector2d) other;
            return Math.abs(x - otherVector.x) < EPSILON && Math.abs(y - otherVector.y) < EPSILON;
        }
        return false;
    }

    @Override
    public String toString() {
        return "<" + x + ", " + y + ">";
    }

    public static double getCosAngle(Vector2d v1, Vector2d v2) {
        double dot = v1.x * v2.x + v1.y * v2.y;
        return dot / (v1.norm() * v2.norm());
    }

    public static double distance(Vector2d v1, Vector2d v2) {
        return v1.added(v2.negated()).norm();
    }
}
