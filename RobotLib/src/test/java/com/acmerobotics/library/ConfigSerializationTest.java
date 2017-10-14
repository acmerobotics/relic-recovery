package com.acmerobotics.library;

import com.acmerobotics.library.dashboard.config.OptionGroup;
import com.acmerobotics.library.dashboard.config.OptionType;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.junit.Test;

/**
 * @author Ryan
 */

public class ConfigSerializationTest {
    public static class A {
        public static double x = 10.3;
        public static boolean y = false;
        public static OptionType type = OptionType.DOUBLE;
        public static PIDCoefficients pid = new PIDCoefficients(0.1, 0.2, 0.5);
    }

    @Test
    public void testSerialization() {
        OptionGroup optionGroup = OptionGroup.createFromClass(A.class);
        System.out.println(RobotDashboard.GSON.toJson(optionGroup));
        System.out.println(optionGroup.getAsJson());
    }
}
