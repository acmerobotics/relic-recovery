package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;

public class SplineSegment implements ParametricPath {
    public static int ARC_LENGTH_SAMPLES = 100000;

    /**
     * These spline parameters are the same as Pathfinder and TrajectoryLib.
     * q(t) = at^5 + bt^4 + ct^3 + dt^2 + et
     */
    private double a, b, c, d, e;
    private double xOffset, yOffset, headingOffset, knotDistance;
    private double length = 1;

    public SplineSegment(Pose2d startPose, Pose2d endPose) {
        xOffset = startPose.x();
        yOffset = startPose.y();

        knotDistance = Vector2d.distance(startPose.pos(), endPose.pos());
        headingOffset = Math.atan2(endPose.y() - startPose.y(), endPose.x() - startPose.x());

        double a0Delta = Math.tan(Angle.norm(startPose.heading() - headingOffset));
        double a1Delta = Math.tan(Angle.norm(endPose.heading() - headingOffset));

        a = 0;
        b = 0;
        c = (a0Delta + a1Delta) / (knotDistance * knotDistance);
        d = -(2 * a0Delta + a1Delta) / knotDistance;
        e = a0Delta;

        computeLength();
    }

    private double derivativeAt(double percentage) {
        double x = knotDistance * percentage;
        return (5*a*x + 4*b) * (x*x*x) + (3*c*x + 2*d) * x + e;
    }

    private double secondDerivativeAt(double percentage) {
        double x = knotDistance * percentage;
        return (20 * a * x + 12 * b) * (x*x) + 6 * c * x + 2 * d;
    }

    private void computeLength() {

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
        double x = knotDistance * percentage;
        double y = (a*x + b) * (x*x*x*x) + c * (x*x*x)+ d * (x*x) + e * x;

        double cosHeadingOffset = Math.cos(headingOffset);
        double sinHeadingOffset = Math.sin(headingOffset);

        double heading = Angle.norm(Math.atan(derivativeAt(percentage) + headingOffset));

        return new Pose2d(
                x * cosHeadingOffset - y * sinHeadingOffset + xOffset,
                x * sinHeadingOffset + y * cosHeadingOffset + yOffset,
                heading
        );
    }

    @Override
    public Pose2d getVelocity(double displacement) {
        return null;
    }

    @Override
    public Pose2d getAcceleration(double displacement) {
        return null;
    }
}
