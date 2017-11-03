package com.acmerobotics.relicrecovery.motion;

import com.acmerobotics.velocityvortex.drive.PIDController;

import org.junit.Test;

import static org.junit.Assert.*;

/**
 * @author Ryan
 */

public class PIDControllerTest {

    @Test
    public void testProportional() {
        PIDController controller = new PIDController(1, 0, 0);
        assertEquals("Proportional isn't computed correctly", 1.0, controller.update(1), 0.0001);
    }

    @Test
    public void testIntegral() {
        PIDController controller = new PIDController();
        double actualSum = 1.0 / 3.0;
        for (int i = 0; i <= 1000; i++) {
            controller.update(Math.pow(i / 1000.0, 2), i / 1000.0);
        }
        assertEquals("Integral isn't computed accurately enough", actualSum, controller.getErrorSum(), 0.0001);
    }

    @Test
    public void testDerivative() {
        PIDController controller = new PIDController();
        controller.update(1, 0);
        controller.update(3, 1);
        assertEquals("Derivative isn't computed correctly", 2, controller.getErrorDerivative(), 0.0001);
    }

    @Test
    public void testIntegralWindup() {
        PIDController controller = new PIDController(0, 1, 0);
        controller.setMaxSum(5);
        controller.update(0, 0);
        assertEquals("Integral error isn't capped at the max sum", 5, controller.update(100, 1), 0.0001);
    }

    @Test
    public void testBoundedError() {
        PIDController controller = new PIDController(1, 0, 0);
        controller.setInputBounds(0, 360);
        double error = controller.getError(340, 60);
        assertEquals("Bounded input error doesn't wrap around", -80, error, 0.0001);
    }

}
