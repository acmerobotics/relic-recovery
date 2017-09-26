package com.acmerobotics.library.localization;

/**
 * Created by kelly on 9/21/2017.
 *
 */

public class Twist {

    private double dx; //in
    private double dy; //in
    private Angle dTheta; //radians

    public Twist(double dx, double dy, Angle dTheta) {
        this.dy = dy;
        this.dx = dx;
        this.dTheta = dTheta;
    }

    public static Twist fromArcHeading(double arcLength, Angle thetaInitial, Angle thetaFinal){
        double direction = Math.signum(arcLength);
        Angle theta = thetaFinal.subtract(thetaInitial);
        double r = Math.abs(arcLength / theta.value());
        Angle chordAngle = new Angle((thetaInitial.value() + thetaFinal.value()) / 2);
        if (Math.abs(thetaInitial.value() - thetaFinal.value()) >= Math.PI) chordAngle = new Angle(chordAngle.value() + Math.PI);
        System.out.println("chord angle: " + (chordAngle.value() / Math.PI + "PI"));
        double chordLength = r * Math.sqrt(2 * (1 - Math.cos(theta.value()))); //law of cosines
        System.out.println("chord length: " + chordLength);
        double dx = Math.cos(chordAngle.value()) * chordLength;
        double dy = Math.sin(chordAngle.value()) * chordLength;
        return new Twist (dx, dy, theta);
    }

    public double dx() {
        return dx;
    }

    public double dy() {
        return dy;
    }

    public Angle dTheta() {
        return dTheta;
    }
}
