package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

public class TrackingOmniLocalizer implements Localizer {
    // position of the omnis in the drive coordinate frame
    public static Vector2d FIRST_WHEEL_POSITION = new Vector2d(0, 0);
    public static Vector2d SECOND_WHEEL_POSITION = new Vector2d(0, 0);

    // the direction the omnis point in (magnitude is ignored)
    public static Vector2d FIRST_WHEEL_DIRECTION = new Vector2d(0, 0);
    public static Vector2d SECOND_WHEEL_DIRECTION = new Vector2d(0, 0);

    public static double OMNI_RADIUS = 0; // TODO

    private MecanumDrive drive;
    private Vector2d estimatedPosition;
    private double firstWheelLastRotation, secondWheelLastRotation, lastHeading;
    private boolean initialized;

    public TrackingOmniLocalizer(MecanumDrive drive) {
        this.drive = drive;
    }

    @Override
    public Vector2d update() {
        double[] trackingOmniRotations = new double[2]; // TODO
        double firstWheelRotation = trackingOmniRotations[0];
        double secondWheelRotation = trackingOmniRotations[1];
        double heading = drive.getHeading();

        if (initialized) {
            double firstWheelDelta = OMNI_RADIUS * (firstWheelRotation - firstWheelLastRotation);
            double secondWheelDelta = OMNI_RADIUS * (secondWheelRotation - secondWheelLastRotation);
            double headingDelta = Angle.norm(heading - lastHeading);

            double deltaY = (FIRST_WHEEL_DIRECTION.x() * secondWheelDelta * SECOND_WHEEL_DIRECTION.norm()
                    - SECOND_WHEEL_DIRECTION.x() * firstWheelDelta * FIRST_WHEEL_DIRECTION.norm()
                    + headingDelta * (SECOND_WHEEL_DIRECTION.x() * SECOND_WHEEL_POSITION.y()
                        - SECOND_WHEEL_DIRECTION.y() * SECOND_WHEEL_POSITION.x()
                        - SECOND_WHEEL_DIRECTION.x() * FIRST_WHEEL_DIRECTION.x() * FIRST_WHEEL_POSITION.y()
                        - SECOND_WHEEL_DIRECTION.x() * FIRST_WHEEL_DIRECTION.y() * FIRST_WHEEL_POSITION.x())) /
                    (FIRST_WHEEL_DIRECTION.y() * SECOND_WHEEL_DIRECTION.x() + SECOND_WHEEL_DIRECTION.y());
            double deltaX = (firstWheelDelta * FIRST_WHEEL_DIRECTION.norm()
                    + FIRST_WHEEL_DIRECTION.y() * deltaY
                    + headingDelta * (FIRST_WHEEL_DIRECTION.x() * FIRST_WHEEL_POSITION.y()
                        + FIRST_WHEEL_DIRECTION.y() * FIRST_WHEEL_POSITION.x())) /
                    FIRST_WHEEL_DIRECTION.x();

            estimatedPosition = estimatedPosition.added(new Vector2d(deltaX, deltaY));
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

    }
}
