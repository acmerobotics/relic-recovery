package com.acmerobotics.library.localization;

import org.firstinspires.ftc.robotcore.external.matrices.OpenGLMatrix;

/**
 * Created by kelly on 9/20/2017.
 *
 */

public class LocaliationTest {

    public static void main(String[] args) {
        double theta = 1.3;
        Twist twist = Twist.fromArcHeading(1, new Angle(theta), new Angle((theta - (.000001))));

        System.out.println(twist.dTheta().value());
        System.out.println(String.format("(%f, %f)", twist.dx(), twist.dy()));
    }

}