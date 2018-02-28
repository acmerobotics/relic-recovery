package com.acmerobotics.library.localization;

import java.util.Locale;

public class Pose2d {

    private final Vector2d pos;
    private final double heading;

    public Pose2d(Vector2d pos, double heading) {
        this.pos = pos;
        this.heading = Angle.norm(heading);
    }

    public Pose2d(double x, double y, double heading) {
        this(new Vector2d(x, y), heading);
    }

    public Vector2d pos() {
        return pos;
    }

    public double x() {
        return pos.x();
    }

    public double y() {
        return pos.y();
    }

    public double heading() {
        return heading;
    }

    public double dist(Pose2d other) {
        return Math.hypot(pos.x() - other.pos.x(), pos.y() - other.pos.y());
    }

    public Pose2d added(Pose2d other) {
        return new Pose2d(pos.added(other.pos), Angle.norm(heading + other.heading));
    }

    public Pose2d multiplied(double scalar) {
        return new Pose2d(scalar * x(), scalar * y(), scalar * heading());
    }

    @Override
    public boolean equals(Object other) {
        if (other instanceof Pose2d) {
            Pose2d otherPose = (Pose2d) other;
            return otherPose.pos().equals(pos) && Math.abs(heading - otherPose.heading()) < Vector2d.EPSILON;
        }
        return false;
    }

    public Pose2d copy() {
        return new Pose2d(pos.copy(), heading);
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH, "<%.2f, %.2f, %.2f (%.2f\u00B0)>", pos.x(), pos.y(), heading, Math.toDegrees(heading));
    }

}