package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

public class TrackingOmniLocalizer implements Localizer {
    // TODO: determine wheel positions and directions
    // position of the omnis in the drive coordinate frame (in)
    public static Vector2d FIRST_WHEEL_POSITION = new Vector2d(0, 0);
    public static Vector2d SECOND_WHEEL_POSITION = new Vector2d(0, 0);

    // the direction the omnis point in (magnitude is ignored)
    public static Vector2d FIRST_WHEEL_DIRECTION = new Vector2d(0, 0);
    public static Vector2d SECOND_WHEEL_DIRECTION = new Vector2d(0, 0);

    // omni radius (in)
    public static double OMNI_RADIUS = 1.1811;

    private MecanumDrive drive;
    private Vector2d estimatedPosition;
    private double firstWheelLastRotation, secondWheelLastRotation, lastHeading;
    private boolean initialized;

    public TrackingOmniLocalizer(MecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public Vector2d update() {
        double[] trackingOmniRotations = drive.getTrackingEncoderRotations();
        double firstWheelRotation = trackingOmniRotations[0];
        double secondWheelRotation = trackingOmniRotations[1];
        double heading = drive.getHeading();

        if (initialized) {
            double firstWheelDelta = OMNI_RADIUS * (firstWheelRotation - firstWheelLastRotation);
            double secondWheelDelta = OMNI_RADIUS * (secondWheelRotation - secondWheelLastRotation);
            double headingDelta = Angle.norm(heading - lastHeading);

            double firstWheelNorm = FIRST_WHEEL_DIRECTION.norm();
            double secondWheelNorm = SECOND_WHEEL_DIRECTION.norm();
            double determinant = FIRST_WHEEL_DIRECTION.x() * SECOND_WHEEL_DIRECTION.y() - FIRST_WHEEL_DIRECTION.y() * SECOND_WHEEL_DIRECTION.x();

            if (Math.abs(determinant) < Vector2d.EPSILON) {
                throw new RuntimeException("The tracking omnis must point in different directions");
            }

            double deltaX = (SECOND_WHEEL_DIRECTION.y() * firstWheelDelta * firstWheelNorm
                    - FIRST_WHEEL_DIRECTION.y() * secondWheelDelta * secondWheelNorm
                    + headingDelta * (FIRST_WHEEL_DIRECTION.x() * SECOND_WHEEL_DIRECTION.y() * FIRST_WHEEL_POSITION.y()
                            + FIRST_WHEEL_DIRECTION.y() * SECOND_WHEEL_DIRECTION.y() * SECOND_WHEEL_POSITION.x()
                            - FIRST_WHEEL_DIRECTION.y() * SECOND_WHEEL_DIRECTION.y() * FIRST_WHEEL_POSITION.x()
                            - FIRST_WHEEL_DIRECTION.y() * SECOND_WHEEL_DIRECTION.x() * SECOND_WHEEL_POSITION.y())) / determinant;
            double deltaY = (FIRST_WHEEL_DIRECTION.x() * secondWheelDelta * secondWheelNorm
                    - SECOND_WHEEL_DIRECTION.x() * firstWheelDelta * firstWheelNorm
                    + headingDelta * (FIRST_WHEEL_DIRECTION.y() * SECOND_WHEEL_DIRECTION.x() * FIRST_WHEEL_POSITION.x()
                            + FIRST_WHEEL_DIRECTION.x() * SECOND_WHEEL_DIRECTION.x() * SECOND_WHEEL_POSITION.y()
                            - FIRST_WHEEL_DIRECTION.x() * SECOND_WHEEL_POSITION.x() * FIRST_WHEEL_POSITION.y()
                            - FIRST_WHEEL_DIRECTION.x() * SECOND_WHEEL_POSITION.y() * SECOND_WHEEL_POSITION.x())) / determinant;

            Vector2d robotPoseDelta = new Vector2d(deltaX, deltaY);
            Vector2d fieldPoseDelta = robotPoseDelta.rotated(heading);
            estimatedPosition = estimatedPosition.added(fieldPoseDelta);
        } else {
            initialized = true;
        }

        firstWheelLastRotation = firstWheelRotation;
        secondWheelLastRotation = secondWheelRotation;
        lastHeading = heading;

        return estimatedPosition;
    }

    @Override
    public void setEstimatedPosition(Vector2d position) {
        estimatedPosition = position;
    }
}
