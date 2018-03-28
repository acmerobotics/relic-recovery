package com.acmerobotics.relicrecovery.path.parametric;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

public class SplinePath implements ParametricPath {
    public enum Type {
        CUBIC_HERMITIAN,
        QUINTIC_HERMITIAN
    }

    public static int ARC_LENGTH_SAMPLES = 100000;

    /**
     * These spline parameters are the same as Pathfinder and TrajectoryLib.
     * q(t) = at^5 + bt^4 + ct^3 + dt^2 + et
     */
    private double a, b, c, d, e;
    private double xOffset, yOffset, headingOffset, knotDistance;
    private double length;
    private boolean headingFlipped;

    public SplinePath(Type type, Pose2d startPose, Pose2d endPose) {
        xOffset = startPose.x();
        yOffset = startPose.y();

        knotDistance = Vector2d.distance(startPose.pos(), endPose.pos());
        headingOffset = Math.atan2(endPose.y() - startPose.y(), endPose.x() - startPose.x());

        double a0Delta = Math.tan(Angle.norm(startPose.heading() - headingOffset));
        double a1Delta = Math.tan(Angle.norm(endPose.heading() - headingOffset));

        if (type == Type.CUBIC_HERMITIAN) {
            a = 0;
            b = 0;
            c = (a0Delta + a1Delta) / (knotDistance * knotDistance);
            d = -(2 * a0Delta + a1Delta) / knotDistance;
            e = a0Delta;
        } else {
            a = -(3 * (a0Delta + a1Delta)) / (knotDistance * knotDistance * knotDistance * knotDistance);
            b = (8 * a0Delta + 7 * a1Delta) / (knotDistance * knotDistance * knotDistance);
            c = -(6 * a0Delta + 4 * a1Delta) / (knotDistance * knotDistance);
            d = 0;
            e = a0Delta;
        }

        computeLength();

        headingFlipped = Math.abs(Angle.norm(startPose.heading() - start().heading())) > Math.PI / 2;
    }

    private double valueAt(double percentage) {
        double x = knotDistance * percentage;
        return (a*x + b) * (x*x*x*x) + c * (x*x*x) + d * (x*x) + e * x;
    }

    private double derivativeAt(double percentage) {
        double x = knotDistance * percentage;
        return (5*a*x + 4*b) * (x*x*x) + (3*c*x + 2*d) * x + e;
    }

    private double secondDerivativeAt(double percentage) {
        double x = knotDistance * percentage;
        return (20*a*x + 12*b) * (x*x) + 6*c * x + 2*d;
    }

    private double thirdDerivativeAt(double percentage) {
        double x = knotDistance * percentage;
        return (60*a*x + 24*b) * x + 6*c;
    }

    private void computeLength() {
        length = 0;
        double lastIntegrand = Math.sqrt(1 + derivativeAt(0) * derivativeAt(0)) / ARC_LENGTH_SAMPLES;
        for (int i = 1; i <= ARC_LENGTH_SAMPLES; i++) {
            double percentage = (double) i / ARC_LENGTH_SAMPLES;
            double dydx = derivativeAt(percentage);
            double integrand = Math.sqrt(1 + dydx * dydx) / ARC_LENGTH_SAMPLES;
            length += (integrand + lastIntegrand) / 2;
            lastIntegrand = integrand;
        }
        length *= knotDistance;
    }

    @Override
    public double length() {
        return length;
    }

    @Override
    public Pose2d start() {
        return getPose(0);
    }

    @Override
    public Pose2d end() {
        return getPose(length);
    }

    @Override
    public Pose2d getPose(double displacement) {
        double percentage = displacement / length;

        double derivative = derivativeAt(percentage);

        double x = knotDistance * percentage;
        double y = valueAt(percentage);
        double heading = Math.atan(derivative) + headingOffset;
        if (headingFlipped) {
            heading += Math.PI;
        }
        heading = Angle.norm(heading);

        return new Pose2d(
                x * Math.cos(headingOffset) - y * Math.sin(headingOffset) + xOffset,
                x * Math.sin(headingOffset) + y * Math.cos(headingOffset) + yOffset,
                heading
        );
    }

    @Override
    public Pose2d getDerivative(double displacement) {
        double percentage = displacement / length;

        double derivative = derivativeAt(percentage);
        double secondDerivative = secondDerivativeAt(percentage);

        double xDeriv = knotDistance / length;
        double yDeriv = derivative * knotDistance / length;
        double omega = secondDerivative / (1 + derivative * derivative);
        omega *= knotDistance / length;

        return new Pose2d(
                xDeriv * Math.cos(headingOffset) - yDeriv * Math.sin(headingOffset),
                xDeriv * Math.sin(headingOffset) + yDeriv * Math.cos(headingOffset),
                omega
        );
    }

    @Override
    public Pose2d getSecondDerivative(double displacement) {
        double percentage = displacement / length;

        double derivative = derivativeAt(percentage);
        double secondDerivative = secondDerivativeAt(percentage);
        double thirdDerivative = thirdDerivativeAt(percentage);

        double xSecondDeriv = 0;
        double ySecondDeriv = secondDerivative * knotDistance * knotDistance / (length * length);
        double alpha = (1 + derivative * derivative) * thirdDerivative - secondDerivative * 2 * derivative * secondDerivative;
        alpha /= (1 + derivative * derivative) * (1 + derivative * derivative);
        alpha *= knotDistance * knotDistance / (length * length);

        return new Pose2d(
                xSecondDeriv * Math.cos(headingOffset) - ySecondDeriv * Math.sin(headingOffset),
                xSecondDeriv * Math.sin(headingOffset) + ySecondDeriv * Math.cos(headingOffset),
                alpha
        );
    }

    public double knotDistance() {
        return knotDistance;
    }

    public double xOffset() {
        return xOffset;
    }

    public double yOffset() {
        return yOffset;
    }

    public double headingOffset() {
        return headingOffset;
    }

    public double a() {
        return a;
    }

    public double b() {
        return b;
    }

    public double c() {
        return c;
    }

    public double d() {
        return d;
    }

    public double e() {
        return e;
    }
}
