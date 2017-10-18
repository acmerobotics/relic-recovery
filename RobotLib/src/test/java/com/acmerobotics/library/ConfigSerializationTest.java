package com.acmerobotics.library;

import com.acmerobotics.library.dashboard.config.OptionGroup;
import com.acmerobotics.library.dashboard.config.OptionType;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.junit.Test;

/**
 * @author Ryan
 */

public class ConfigSerializationTest {
    public static class A {
        public PIDCoefficients coefficients;

        public A() {
            this.coefficients = new PIDCoefficients(0.2, 0.44, 0.15);
        }
    }

    public static class B {
        public static double x = 10.3;
        public static boolean y = false;
        public static OptionType optionType = OptionType.DOUBLE;
        public static PIDCoefficients pid = new PIDCoefficients(0.1, 0.2, 0.5);
        public static A a = new A();
    }

    @Test
    public void testSerialization() {
        OptionGroup optionGroup = OptionGroup.createFromClass(B.class);
        System.out.println(optionGroup.getJsonSchema());
        System.out.println(optionGroup.getJson());
        JsonObject changeObj = new JsonObject();
        changeObj.add("x", new JsonPrimitive(2.1));
        optionGroup.updateJson(changeObj);
        System.out.println(optionGroup.getJson());
    }
}
