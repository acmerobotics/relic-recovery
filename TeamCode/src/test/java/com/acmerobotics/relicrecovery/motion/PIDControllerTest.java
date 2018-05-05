package com.acmerobotics.relicrecovery.motion;

import com.acmerobotics.library.motion.PIDController;

import org.junit.Test;

import static org.junit.Assert.*;

public class PIDControllerTest {

    @Test
    public void testProportional() {
        PIDController controller = new PIDController(1, 0, 0);
        assertEquals(1.0, controller.update(1), 0.0001);
    }

    @Test
    public void testIntegral() {
        PIDController controller = new PIDController();
        double expectedSum = 1.0 / 3.0;
        for (int i = 0; i <= 1000; i++) {
            controller.update(Math.pow(i / 1000.0, 2), i / 1000.0);
        }
        assertEquals(expectedSum, controller.getErrorSum(), 0.0001);
    }

    @Test
    public void testDerivative() {
        PIDController controller = new PIDController();
        controller.update(1, 0);
        controller.update(3, 1);
        assertEquals(2, controller.getErrorDerivative(), 0.0001);
    }

    @Test
    public void testIntegralWindup() {
        PIDController controller = new PIDController(0, 1, 0);
        controller.setMaxSum(5);
        controller.update(0, 0);
        assertEquals(5, controller.update(100, 1), 0.0001);
    }

    @Test
    public void testBoundedError() {
        PIDController controller = new PIDController(1, 0, 0);
        controller.setInputBounds(0, 360);
        controller.setSetpoint(60);
        double error = controller.getError(340);
        assertEquals(-80, error, 0.0001);
    }

}
