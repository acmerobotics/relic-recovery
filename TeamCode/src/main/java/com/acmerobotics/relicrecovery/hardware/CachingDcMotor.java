package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;

public class CachingDcMotor implements DcMotor {
    private DcMotor delegate;
    private double cachedPower;

    public CachingDcMotor(DcMotor delegate) {
        this.delegate = delegate;
    }

    @Override
    public MotorConfigurationType getMotorType() {
        return delegate.getMotorType();
    }

    @Override
    public void setMotorType(MotorConfigurationType motorType) {
        delegate.setMotorType(motorType);
    }

    @Override
    public DcMotorController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
    }

    @Override
    public void setZeroPowerBehavior(ZeroPowerBehavior zeroPowerBehavior) {
        delegate.setZeroPowerBehavior(zeroPowerBehavior);
    }

    @Override
    public ZeroPowerBehavior getZeroPowerBehavior() {
        return delegate.getZeroPowerBehavior();
    }

    @Override
    public void setPowerFloat() {
        delegate.setPowerFloat();
    }

    @Override
    public boolean getPowerFloat() {
        return delegate.getPowerFloat();
    }

    @Override
    public void setTargetPosition(int position) {
        delegate.setTargetPosition(position);
    }

    @Override
    public int getTargetPosition() {
        return delegate.getTargetPosition();
    }

    @Override
    public boolean isBusy() {
        return delegate.isBusy();
    }

    @Override
    public int getCurrentPosition() {
        return delegate.getCurrentPosition();
    }

    @Override
    public void setMode(RunMode mode) {
        delegate.setMode(mode);
    }

    @Override
    public RunMode getMode() {
        return delegate.getMode();
    }

    @Override
    public void setDirection(Direction direction) {
        delegate.setDirection(direction);
    }

    @Override
    public Direction getDirection() {
        return delegate.getDirection();
    }

    @Override
    public void setPower(double power) {
        if (power != cachedPower) {
            delegate.setPower(power);
            cachedPower = power;
        }
    }

    @Override
    public double getPower() {
        return delegate.getPower();
    }

    @Override
    public Manufacturer getManufacturer() {
        return delegate.getManufacturer();
    }

    @Override
    public String getDeviceName() {
        return delegate.getDeviceName();
    }

    @Override
    public String getConnectionInfo() {
        return delegate.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return delegate.getVersion();
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        delegate.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        delegate.close();
    }
}
