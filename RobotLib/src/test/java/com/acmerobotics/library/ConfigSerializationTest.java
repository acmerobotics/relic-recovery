package com.acmerobotics.library;

import com.acmerobotics.library.dashboard.config.Configuration;
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
        Configuration config = new Configuration();
        config.addOptionsFromClass(B.class);
        System.out.println(config.getJsonSchema());
        System.out.println(config.getJson());
        JsonObject changeObj = new JsonObject();
        JsonObject bObj = new JsonObject();
        JsonObject pidObj = new JsonObject();
        pidObj.add("i", new JsonPrimitive(27.3));
        bObj.add("x", new JsonPrimitive(2.1));
        bObj.add("pid", pidObj);
        changeObj.add("B", bObj);
        config.updateJson(changeObj);
        System.out.println(config.getJson());
    }
}
