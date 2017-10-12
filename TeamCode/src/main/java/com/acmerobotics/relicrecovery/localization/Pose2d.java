package com.acmerobotics.relicrecovery.localization;

/**
 * Created by kelly on 9/28/2017.
 *
 */

public class Pose2d {

    private Vector2d pos;
    private double heading;

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

}