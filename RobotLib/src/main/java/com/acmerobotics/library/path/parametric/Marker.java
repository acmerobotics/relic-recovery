package com.acmerobotics.library.path.parametric;

import com.acmerobotics.library.localization.Pose2d;

public class Marker implements ParametricPath {
    private String name;
    private Pose2d pose;

    public Marker(String name, Pose2d pose) {
        this.name = name;
        this.pose = pose;
    }

    @Override
    public double length() {
        return 0;
    }

    @Override
    public Pose2d start() {
        return pose;
    }

    @Override
    public Pose2d end() {
        return pose;
    }

    @Override
    public Pose2d getPose(double displacement) {
        return pose;
    }

    @Override
    public Pose2d getDerivative(double displacement) {
        return new Pose2d(0, 0, 0);
    }

    @Override
    public Pose2d getSecondDerivative(double displacement) {
        return new Pose2d(0, 0, 0);
    }

    public String getName() {
        return name;
    }
}
