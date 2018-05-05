package com.acmerobotics.library.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SharpGP2Y0A51SK0FProximitySensor implements HardwareDevice {
    // TODO: are the parameters constant for all surfaces?
    public enum Surface {
        CRYPTO(7.26557, -1.49673, -0.264094, 2.54, 1.55966);

        /**
         * Linearization parameters:
         * y = ae^(bx) + c where x is voltage and y is *raw distance*
         * y = dx + f where x is raw distance and y is normalized distance (cm)
         */
        private double a, b, c, d, f;

        Surface(double a, double b, double c, double d, double f) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.f = f;
        }

        private double getRawDistance(double voltage) {
            return a * Math.exp(b * voltage) + c;
        }

        public double getDistance(double voltage, DistanceUnit unit) {
            double rawDistance = getRawDistance(voltage);
            double distanceCm = d * rawDistance + f;
            return unit.fromCm(distanceCm);
        }
    }

    private AnalogInput input;

    public SharpGP2Y0A51SK0FProximitySensor(AnalogInput analogInput) {
        input = analogInput;
    }

    public double getRawDistance(Surface surface) {
        return surface.getRawDistance(input.getVoltage());
    }

    public double getDistance(Surface surface, DistanceUnit unit) {
        return surface.getDistance(input.getVoltage(), unit);
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Other;
    }

    @Override
    public String getDeviceName() {
        return "Sharp GP2Y0A51SK0F Proximity Sensor";
    }

    @Override
    public String getConnectionInfo() {
        return input.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 1;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {
        input.resetDeviceConfigurationForOpMode();
    }

    @Override
    public void close() {
        input.close();
    }
}
