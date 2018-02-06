package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class UltrasonicLocalizer extends DeadReckoningLocalizer {
    public static final double SENSOR_OFFSET = 8.5; // in

    private boolean useUltrasonicFeedback;
    private ExponentialSmoother ultrasonicSmoother;

    public UltrasonicLocalizer(MecanumDrive drive, double smoothingCoeff) {
        super(drive);

        ultrasonicSmoother = new ExponentialSmoother(smoothingCoeff);
    }

    public void enableUltrasonicFeedback() {
        ultrasonicSmoother.reset();
        useUltrasonicFeedback = true;
    }

    public void disableUltrasonicFeedback() {
        useUltrasonicFeedback = false;
    }

    @Override
    public Vector2d update() {
        Vector2d estimatedPosition = super.update();

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
            double ultrasonicDistance = ultrasonicSmoother.update(drive.getUltrasonicDistance(DistanceUnit.INCH));
            if (ultrasonicDistance > drive.getMinUltrasonicDistance(DistanceUnit.INCH)) {
                switch (closestCryptobox) {
                    case NEAR_BLUE:
                        estimatedPosition = new Vector2d(estimatedPosition.x(), -72 + AutoPaths.CRYPTO_COL_DEPTH + ultrasonicDistance + SENSOR_OFFSET);
                        break;
                    case NEAR_RED:
                        estimatedPosition = new Vector2d(estimatedPosition.x(), 72 - AutoPaths.CRYPTO_COL_DEPTH - ultrasonicDistance - SENSOR_OFFSET);
                        break;
                    case FAR_BLUE:
                    case FAR_RED:
                        estimatedPosition = new Vector2d(-72 + AutoPaths.CRYPTO_COL_DEPTH + ultrasonicDistance + SENSOR_OFFSET, estimatedPosition.y());
                        break;
                }
            }
        }

        return estimatedPosition;
    }
}
