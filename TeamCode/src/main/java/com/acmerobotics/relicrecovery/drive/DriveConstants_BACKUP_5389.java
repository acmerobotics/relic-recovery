package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

@Config
public class DriveConstants {
    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(24.0, 48.0, 48.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 4.0, 4.0, MotionConstraints.EndBehavior.OVERSHOOT);

    public static PIDFCoefficients HEADING_PID = new PIDFCoefficients(0, 0, 0, 0.234, 0);
    public static PIDFCoefficients AXIAL_PID = new PIDFCoefficients(0, 0, 0, 0.0185, 0);
    public static PIDFCoefficients LATERAL_PID = new PIDFCoefficients(0, 0, 0, 0.0183, 0);
    
    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double RAMP_MAX_ACCEL = 25;
}
