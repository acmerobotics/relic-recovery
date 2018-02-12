package com.acmerobotics.library.localization;

public class Angle {

    public static final double TAU = Math.PI * 2;

    public static double norm(double angle) {
        angle =  angle % TAU;

        angle = (angle + TAU) % TAU;

        if (angle > Math.PI) {
            angle -= TAU;
        }

        return angle;
    }

}