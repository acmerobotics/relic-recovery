package com.acmerobotics.relicrecovery.motion;

import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.util.Range;

import java.util.Locale;

/**
 * @author Ryan
 */

public class PIDFController {
    private PIDController controller;
    private PIDFCoefficients coeff;
    private MotionState setpoint;

    public PIDFController(PIDFCoefficients coeff) {
        this.coeff = coeff;
        this.controller = new PIDController(coeff);
    }

    public MotionState getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(MotionState setpoint) {
        this.controller.setSetpoint(setpoint.x);
        this.setpoint = setpoint;
    }

    public double getPositionError(double actual) {
        return controller.getError(actual);
    }

    public double update(double positionError) {
        return update(positionError, System.nanoTime() / Math.pow(10, 9));
    }

    public double update(double positionError, double time) {
        double update = controller.update(positionError, time);
        double feedforward = coeff.v * setpoint.v + coeff.a * setpoint.a;
        double output = update + feedforward;
        if (controller.isOutputBounded()) {
            return Range.clip(output, getMinOutput(), getMaxOutput());
        }
        return output;
    }

    public void setOutputBounds(double min, double max) {
        controller.setOutputBounds(min, max);
    }

    public double getMinOutput() {
        return controller.getMinOutput();
    }

    public double getMaxOutput() {
        return controller.getMaxOutput();
    }

    public boolean isOutputBounded() {
        return controller.isOutputBounded();
    }

    public void clearOutputBounds() {
        controller.clearOutputBounds();
    }

    public void setInputBounds(double min, double max) {
        controller.setInputBounds(min, max);
    }

    public double getMinInput() {
        return controller.getMinInput();
    }

    public double getMaxInput() {
        return controller.getMaxInput();
    }

    public boolean isInputBounded() {
        return controller.isInputBounded();
    }

    public void clearInputBounds() {
        controller.clearInputBounds();
    }

    public void setMaxSum(double max) {
        controller.setMaxSum(max);
    }

    public double getMaxSum() {
        return controller.getMaxSum();
    }

    public void reset() {
        controller.reset();
    }

    public double getErrorSum() {
        return controller.getErrorSum();
    }

    public double getErrorDerivative() {
        return controller.getErrorDerivative();
    }

    public PIDCoefficients getCoefficients() {
        return coeff;
    }

    @Override
    public String toString() {
        return String.format(Locale.ENGLISH,"(%4.3f, %4.3f, %4.3f, %4.3f, %4.3f)", coeff.p, coeff.i, coeff.d, coeff.v, coeff.a);
    }

}
