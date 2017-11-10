package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.qualcomm.hardware.motors.NeveRest60Gearmotor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

/**
 * Created by ryanbrott on 10/28/17.
 */

@Config
public class DriveConstants {
    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(16.0, 16.0, 16.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(1.5, 1.5, 1.5, MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_V);

    public static PIDFCoefficients HEADING_COEFFS = new PIDFCoefficients(0, 0, 0, 1 / 2.44, 0);
    public static PIDFCoefficients AXIAL_COEFFS = new PIDFCoefficients(0, 0, 0, 1 / 22.0, 0);
    public static PIDCoefficients LATERAL_COEFFS = new PIDCoefficients(0, 0, 0);
}
