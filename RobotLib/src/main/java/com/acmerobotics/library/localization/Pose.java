package com.acmerobotics.library.localization;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;
import org.firstinspires.ftc.robotcore.external.matrices.VectorF;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

/**
 * Created by kelly on 9/20/2017.
 *
 */

public class Pose {

    private double x; //in
    private double y; //in
    private Angle theta; //radians

    /**
     *
     * @param x inches
     * @param y inches
     * @param theta radians
     */
    public Pose(double x, double y, Angle theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
    }

    public void addTwist (Twist twist) {
        x += twist.dx();
        y += twist.dy();
        theta = theta.add(twist.dTheta());
    }

    public String toString() {
        return "(" + x + ", " + y + ") " + theta;
    }

    public static Pose matrixToPose(OpenGLMatrix matrix) {
        VectorF translation = matrix.getTranslation();
        Orientation orientation = Orientation.getOrientation(matrix, AxesReference.EXTRINSIC, AxesOrder.XYZ, AngleUnit.RADIANS);
        float x = translation.get(0);
        float y = translation.get(1);
        float heading = orientation.thirdAngle;
        return new Pose(x, y, new Angle(heading));
    }

    public double x() {
        return x;
    }

    public double y() {
        return y;
    }

    public Angle theta() {
        return theta;
    }

}
