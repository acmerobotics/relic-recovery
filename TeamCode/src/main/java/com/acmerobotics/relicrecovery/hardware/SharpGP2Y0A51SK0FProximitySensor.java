package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.HardwareDevice;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class SharpGP2Y0A51SK0FProximitySensor implements HardwareDevice {
    public enum Surface {
        // TODO: determine e and f
        RED_CRYPTO(-68.5004, 18.2733, -1.85815, 45.6475, 1, 0),
        BLUE_CRYPTO(1.57038, 0.102239, -5.10007, 0.0576782, 1, 0);

        /**
         * Linearization parameters:
         * y = a + b * ln(x^c + d) where x is voltage and y is *raw distance*
         * y = ex + f where x is raw distance and y is normalized distance (in cm)
         */
        private double a, b, c, d, e, f;

        Surface(double a, double b, double c, double d, double e, double f) {
            this.a = a;
            this.b = b;
            this.c = c;
            this.d = d;
            this.e = e;
            this.f = f;
        }

        private double getRawDistance(double voltage) {
            return a + b * Math.log(Math.pow(voltage, c) + d);
        }

        public double getDistance(double voltage, DistanceUnit unit) {
            double rawDistance = getRawDistance(voltage);
            double distanceCm = e * rawDistance + f;
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
