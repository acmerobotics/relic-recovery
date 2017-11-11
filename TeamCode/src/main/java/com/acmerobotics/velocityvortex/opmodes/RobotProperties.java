package com.acmerobotics.velocityvortex.opmodes;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * Created by ryanbrott on 8/26/17.
 */

public class RobotProperties {
    public static PIDCoefficients turnPid = new PIDCoefficients(0.025, 0, 0);
    public static PIDCoefficients pusherPid = new PIDCoefficients(0.5, 0, 0);
}
