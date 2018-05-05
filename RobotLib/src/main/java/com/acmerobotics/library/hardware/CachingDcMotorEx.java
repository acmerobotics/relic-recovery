package com.acmerobotics.library.hardware;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

public class CachingDcMotorEx extends CachingDcMotor implements DcMotorEx {
    private DcMotorEx delegate;

    public CachingDcMotorEx(DcMotorEx delegate) {
        super(delegate);

        this.delegate = delegate;
    }

    @Override
    public void setMotorEnable() {
        delegate.setMotorEnable();
    }

    @Override
    public void setMotorDisable() {
        delegate.setMotorDisable();
    }

    @Override
    public boolean isMotorEnabled() {
        return delegate.isMotorEnabled();
    }

    @Override
    public void setVelocity(double angularRate, AngleUnit unit) {
        delegate.setVelocity(angularRate, unit);
    }

    @Override
    public double getVelocity(AngleUnit unit) {
        return delegate.getVelocity(unit);
    }

    @Override
    public void setPIDCoefficients(RunMode mode, PIDCoefficients pidCoefficients) {
        delegate.setPIDCoefficients(mode, pidCoefficients);
    }

    @Override
    public PIDCoefficients getPIDCoefficients(RunMode mode) {
        return delegate.getPIDCoefficients(mode);
    }

    @Override
    public void setTargetPositionTolerance(int tolerance) {
        delegate.setTargetPositionTolerance(tolerance);
    }

    @Override
    public int getTargetPositionTolerance() {
        return delegate.getTargetPositionTolerance();
    }
}
