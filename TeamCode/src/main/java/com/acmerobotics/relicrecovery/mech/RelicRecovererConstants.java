package com.acmerobotics.relicrecovery.mech;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by kelly on 11/13/2017.
 *
 */

@Config
public class RelicRecovererConstants {
    public static PIDCoefficients EXTENSION_COEFFICIENTS = new PIDCoefficients(-.002, 0, 0);
    public static PIDCoefficients OFFSET_COEFFICIENTS = new PIDCoefficients(-.001, 0, 0);
    public static double MAX_EXTENSION_CORRECTION = .8;
    public static double MAX_EXTENSION = 10000000000000.0;
    public static double OPEN_OFFSET = -100;
    public static double UP_OFFSET = 100;
}
