package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class UltrasonicLocalizer extends DeadReckoningLocalizer {
    public static double SMOOTHING_COEFF = 0.1;

    // linear regression: y = ax + b
    // y: actual distance, x: raw ultrasonic distance
    public static double COEFFICIENT_A = 0.923;
    public static double COEFFICIENT_B = 6.152;

    public static double MIN_DISTANCE = 0;

    /** the weight of the ultrasonic readings in the complementary filter */
    public static double ULTRASONIC_WEIGHT = 1;

    private boolean useUltrasonicFeedback;
    private ExponentialSmoother ultrasonicSmoother;
    private double ultrasonicDistance;

    public UltrasonicLocalizer(MecanumDrive drive) {
        super(drive);

        ultrasonicSmoother = new ExponentialSmoother(SMOOTHING_COEFF);
    }

    public void enableUltrasonicFeedback() {
        useUltrasonicFeedback = true;
    }

    public void disableUltrasonicFeedback() {
        useUltrasonicFeedback = false;
    }

    /** complementary filter action */
    public double combineEstimates(double deadReckoningEstimate, double ultrasonicEstimate) {
        return (1 - ULTRASONIC_WEIGHT) * deadReckoningEstimate + ULTRASONIC_WEIGHT * ultrasonicEstimate;
    }

    @Override
    public Vector2d update() {
        super.update();

        double rawDistance = ultrasonicSmoother.update(drive.getUltrasonicDistance(DistanceUnit.INCH));

        if (rawDistance <= MIN_DISTANCE) {
            return estimatedPosition;
        }

        ultrasonicDistance = COEFFICIENT_A * rawDistance + COEFFICIENT_B;

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

            if (ultrasonicDistance > drive.getMinUltrasonicDistance(DistanceUnit.INCH)) {
                switch (closestCryptobox) {
                    case NEAR_BLUE:
                        estimatedPosition = new Vector2d(estimatedPosition.x(),
                                combineEstimates(estimatedPosition.y(), -71 + ultrasonicDistance));
                        break;
                    case NEAR_RED:
                        estimatedPosition = new Vector2d(estimatedPosition.x(),
                                combineEstimates(estimatedPosition.y(), 71 - ultrasonicDistance));
                        break;
                    case FAR_BLUE:
                    case FAR_RED:
                        estimatedPosition = new Vector2d(combineEstimates(estimatedPosition.x(),
                                -71 + ultrasonicDistance), estimatedPosition.y());
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
