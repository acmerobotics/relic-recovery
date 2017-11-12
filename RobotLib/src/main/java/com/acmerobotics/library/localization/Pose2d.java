package com.acmerobotics.library.localization;

import java.util.Locale;

/**
 * Created by kelly on 9/28/2017.
 *
 */

public class Pose2d {

    private Vector2d pos;
    private double heading;

    public Pose2d(Vector2d pos) {
        this(pos, 0);
    }

    public Pose2d(Vector2d pos, double heading) {
        this.pos = pos;
        this.heading = Angle.norm(heading);
    }

    public Pose2d(double x, double y) {
        this(x, y, 0);
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

    public void add(Pose2d other) {
        pos.add(other.pos);
        heading = Angle.norm(heading + other.heading);
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
        return String.format(Locale.ENGLISH, "<%.2f, %.2f, %.2f (%.2f deg)>", pos.x(), pos.y(), heading, Math.toDegrees(heading));
    }

}