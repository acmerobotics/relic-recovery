package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by ryanbrott on 10/28/17.
 */

@Config
public class DriveConstants {
    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(0, 0, 0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(0, 0, 0, MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_V);

    public static PIDFCoefficients HEADING_COEFFS = new PIDFCoefficients();
    public static PIDFCoefficients AXIAL_COEFFS = new PIDFCoefficients();
    public static PIDCoefficients LATERAL_COEFFS = new PIDCoefficients();
}
