package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;

/**
 * Created by kelly on 9/28/2017.
 *
 */

public class PoseDelta2d {

    private Vector2d dPos;
    private double dHeading;

    public PoseDelta2d(Vector2d dPos, double dHeading) {
        this.dPos = dPos;
        this.dHeading = dHeading;
    }

    public PoseDelta2d(double dX, double dY, double dHeading) {
        this.dPos = new Vector2d(dX, dY);
        this.dHeading = dHeading;
    }

    /*
    public static PoseDelta2d fromMecanum(int[] rot) {
        double x = ( rot[0] + rot[1] + rot[2] + rot[3]) / 4;
        double y = (-rot[0] + rot[1] - rot[2] + rot[3]) / 4;
        double h = (-rot[0] - rot[1] + rot[2] + rot[3]) / (4 * MecanumDrive.K);

    }*/

    public Vector2d dPos() {
        return dPos;
    }

    public double dX() {
        return dPos.x();
    }

    public double dY() {
        return dPos.y();
    }

    public double dHeading() {
        return dHeading;
    }

}
