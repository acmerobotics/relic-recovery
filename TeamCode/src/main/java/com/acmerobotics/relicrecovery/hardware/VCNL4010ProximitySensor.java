package com.acmerobotics.relicrecovery.hardware;

import android.support.annotation.NonNull;

import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDeviceWithParameters;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.hardware.configuration.I2cSensor;
import com.qualcomm.robotcore.util.TypeConversion;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.nio.ByteOrder;

@I2cSensor(name = "VCNL4010 Proximity Sensor", description = "Proximity Sensor from Adafruit", xmlTag = "VCNL4010")
public class VCNL4010ProximitySensor extends I2cDeviceSynchDeviceWithParameters<I2cDeviceSynch, VCNL4010ProximitySensor.Parameters> implements DistanceSensor {
    public static class Parameters {
        public I2cAddr i2cAddr = I2CADDR_DEFAULT;
        public boolean measureAmbientPeriodic = false;
        public boolean measureProximityPeriodic = true;
        public ProximityRate proximityRate = ProximityRate.RATE_62_5;
        public Frequency frequency = Frequency.FREQ_390_625;
        public int ledCurrent = 20;
    }

    public static final I2cAddr I2CADDR_DEFAULT = I2cAddr.create7bit(0x13);

    public static final byte bMEASURE_AMBIENT = 0x10;
    public static final byte bMEASURE_PROXIMITY = 0x08;
    public static final byte bAMBIENT_READY = 0x40;
    public static final byte bPROXIMITY_READY = 0x20;
    public static final byte bAMBIENT_PERIODIC = 0x04;
    public static final byte bPROXIMITY_PERIODIC = 0x02;
    public static final byte bSELF_TIMED = 0x01;

    /** proximity reads per sec */
    public enum ProximityRate {
        RATE_1_95(0),
        RATE_3_90625(1),
        RATE_7_8125(2),
        RATE_16_625(3),
        RATE_31_25(4),
        RATE_62_5(5),
        RATE_125(6),
        RATE_250(7);

        public final byte bVal;
        ProximityRate(int i) {
            this.bVal = (byte) i;
        }
    }

    /** frequency of the IR test signal */
    public enum Frequency {
        FREQ_390_625(0),
        FREQ_781_25(1),
        FREQ_1_5625(2),
        FREQ_3_125(3);

        public final byte bVal;
        Frequency(int i) {
            this.bVal = (byte) i;
        }
    }

    public enum Register {
        COMMAND(0x80),
        PRODUCT_ID(0x81),
        PROX_RATE(0x82),
        IRLED(0x83),
        AMBIENT_PARAMETER(0x84),
        AMBIENT_DATA(0x85),
        PROXIMITY_DATA(0x87),
        INT_CONTROL(0x89),
        PROXIMITY_ADJUST(0x8A),
        INT_STAT(0x8E),
        MOD_TIMING(0x8F);

        public final byte bVal;
        Register(int i) {
            this.bVal = (byte) i;
        }
    }

    public VCNL4010ProximitySensor(I2cDeviceSynch i2cDeviceSynch) {
        super(i2cDeviceSynch, true, new Parameters());

        i2cDeviceSynch.engage();
    }

    @Override
    protected boolean internalInitialize(@NonNull Parameters parameters) {
        this.deviceClient.setI2cAddress(parameters.i2cAddr);

        byte rev = read8(Register.PRODUCT_ID);
        if ((rev & 0xF0) != 0x20) {
            return false;
        }

        int commandValue = 0;
        if (parameters.measureProximityPeriodic | parameters.measureAmbientPeriodic) {
            commandValue |= bSELF_TIMED;
        }
        if (parameters.measureProximityPeriodic) {
            commandValue |= bPROXIMITY_PERIODIC;
        }
        if (parameters.measureAmbientPeriodic) {
            commandValue |= bAMBIENT_PERIODIC;
        }
        write8(Register.COMMAND, commandValue);
        waitForWriteCompletions();

        setLEDCurrent(parameters.ledCurrent);
        setProximityRate(parameters.proximityRate);
        setFrequency(parameters.frequency);

        return true;
    }

    public void setLEDCurrent(int current10mA) {
        if (current10mA > 20) {
            current10mA = 20;
        }

        write8(Register.IRLED, current10mA);
        waitForWriteCompletions();
    }

    public void setProximityRate(ProximityRate proximityRate) {
        write8(Register.PROX_RATE, proximityRate.bVal);
        waitForWriteCompletions();
    }

    public void setFrequency(Frequency frequency) {
        write8(Register.MOD_TIMING, (frequency.bVal << 3) | 1);
        waitForWriteCompletions();
    }

    public int readProximity() {
        if (!parameters.measureProximityPeriodic) {
            if (parameters.measureAmbientPeriodic) {
                throw new UnsupportedOperationException("Periodic proximity measurements must be enabled");
            }
            byte i = read8(Register.COMMAND);
            write8(Register.COMMAND, i | bMEASURE_PROXIMITY);
            waitForWriteCompletions();

            while (!Thread.currentThread().isInterrupted()) {
                int result = read8(Register.COMMAND);
                if ((result & bPROXIMITY_READY) != 0) {
                    return readShort(Register.PROXIMITY_DATA);
                }
            }
        } else {
            return readShort(Register.PROXIMITY_DATA);
        }
        return 0;
    }

    public int readAmbient() {
        if (!parameters.measureAmbientPeriodic) {
            if (parameters.measureProximityPeriodic) {
                throw new UnsupportedOperationException("Periodic ambient measurements must be enabled");
            }
            byte i = read8(Register.COMMAND);
            write8(Register.COMMAND, i | bMEASURE_AMBIENT);
            waitForWriteCompletions();

            while (!Thread.currentThread().isInterrupted()) {
                int result = read8(Register.COMMAND);
                if ((result & bAMBIENT_READY) != 0) {
                    return readShort(Register.AMBIENT_DATA);
                }
            }
        } else {
            return readShort(Register.AMBIENT_DATA);
        }
        return 0;
    }

    @Override
    public double getDistance(DistanceUnit unit) {
        // TODO: calibrate this!!
        return readProximity();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "VCNL4010 Proximity Sensor";
    }

    public byte read8(Register reg) {
        return deviceClient.read8(reg.bVal);
    }

    public byte[] read(Register reg, int cb) {
        return deviceClient.read(reg.bVal, cb);
    }

    public int readShort(Register reg) {
        byte[] data = read(reg, 2);
        return TypeConversion.byteArrayToShort(data, ByteOrder.LITTLE_ENDIAN) & 0xFFFF;
    }

    public void write8(Register reg, int data) {
        deviceClient.write8(reg.bVal, data);
    }

    public void write(Register reg, byte[] data) {
        deviceClient.write(reg.bVal, data);
    }

    public void waitForWriteCompletions() {
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }
}
