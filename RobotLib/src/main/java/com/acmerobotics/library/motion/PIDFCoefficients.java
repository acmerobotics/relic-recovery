package com.acmerobotics.library.motion;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

public class PIDFCoefficients extends PIDCoefficients {
    public double v, a;

    public PIDFCoefficients() {
        this(0, 0, 0, 0, 0);
    }

    public PIDFCoefficients(double p, double i, double d, double v, double a) {
        super(p, i, d);
        this.v = v;
        this.a = a;
    }
}
