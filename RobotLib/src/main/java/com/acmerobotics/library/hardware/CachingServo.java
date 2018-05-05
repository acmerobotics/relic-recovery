package com.acmerobotics.library.hardware;

import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoController;

public class CachingServo implements Servo {
    private Servo delegate;
    private double cachedPosition;

    public CachingServo(Servo delegate) {
        this.delegate = delegate;
    }

    @Override
    public ServoController getController() {
        return delegate.getController();
    }

    @Override
    public int getPortNumber() {
        return delegate.getPortNumber();
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
    public void setPosition(double position) {
        if (position != cachedPosition) {
            delegate.setPosition(position);
            cachedPosition = position;
        }
    }

    @Override
    public double getPosition() {
        return delegate.getPosition();
    }

    @Override
    public void scaleRange(double min, double max) {
        delegate.scaleRange(min, max);
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
