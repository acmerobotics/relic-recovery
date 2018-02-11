package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class UltrasonicLocalizer extends DeadReckoningLocalizer {
    public static double MOUNTING_OFFSET = 1.5; // in
    public static double SMOOTHING_COEFF = 0.4;

    // all offsets are measured in reference to the wall
    public static double WALL_OFFSET = 0;
    public static double FULL_COLUMN_OFFSET = 6;
    public static double EMPTY_COLUMN_OFFSET = 2; // in

    public enum UltrasonicTarget {
        WALL,
        FULL_COLUMN,
        EMPTY_COLUMN
    }

    private boolean useUltrasonicFeedback;
    private ExponentialSmoother ultrasonicSmoother;
    private double ultrasonicDistance;
    private UltrasonicTarget target = UltrasonicTarget.WALL;

    public UltrasonicLocalizer(MecanumDrive drive) {
        super(drive);

        ultrasonicSmoother = new ExponentialSmoother(SMOOTHING_COEFF);
    }

    public void setTarget(UltrasonicTarget target) {
        this.target = target;
    }

    public void enableUltrasonicFeedback() {
        useUltrasonicFeedback = true;
    }

    public void disableUltrasonicFeedback() {
        useUltrasonicFeedback = false;
    }

    @Override
    public Vector2d update() {
        Vector2d estimatedPosition = super.update();

        ultrasonicDistance = ultrasonicSmoother.update(drive.getUltrasonicDistance(DistanceUnit.INCH));

        if (useUltrasonicFeedback) {
            Cryptobox closestCryptobox = Cryptobox.NEAR_BLUE;
            double closestDistance = Double.POSITIVE_INFINITY;
            for (Cryptobox cryptobox : Cryptobox.values()) {
                double distance = Vector2d.distance(cryptobox.getPosition(), estimatedPosition);
                if (distance < closestDistance) {
                    closestCryptobox = cryptobox;
                    closestDistance = distance;
                }
            }

            double targetOffset;
            switch (target) {
                case WALL:
                    targetOffset = WALL_OFFSET;
                    break;
                case FULL_COLUMN:
                    targetOffset = FULL_COLUMN_OFFSET;
                    break;
                case EMPTY_COLUMN:
                    targetOffset = EMPTY_COLUMN_OFFSET;
                    break;
                default:
                    targetOffset = 0;
            }

            if (ultrasonicDistance > drive.getMinUltrasonicDistance(DistanceUnit.INCH)) {
                switch (closestCryptobox) {
                    case NEAR_BLUE:
                        estimatedPosition = new Vector2d(estimatedPosition.x(), -72 + targetOffset + ultrasonicDistance + MOUNTING_OFFSET);
                        break;
                    case NEAR_RED:
                        estimatedPosition = new Vector2d(estimatedPosition.x(), 72 - targetOffset - ultrasonicDistance - MOUNTING_OFFSET);
                        break;
                    case FAR_BLUE:
                    case FAR_RED:
                        estimatedPosition = new Vector2d(-72 + EMPTY_COLUMN_OFFSET + targetOffset + MOUNTING_OFFSET, estimatedPosition.y());
                        break;
                }
            }
        }

        return estimatedPosition;
    }

    public double getUltrasonicDistance(DistanceUnit unit) {
        return unit.fromInches(ultrasonicDistance);
    }
}
