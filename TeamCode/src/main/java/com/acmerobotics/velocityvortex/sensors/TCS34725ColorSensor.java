package com.acmerobotics.velocityvortex.sensors;

import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.I2cAddr;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchDevice;
import com.qualcomm.robotcore.hardware.I2cWaitControl;
import com.qualcomm.robotcore.util.TypeConversion;

import java.nio.ByteOrder;

public class TCS34725ColorSensor extends I2cDeviceSynchDevice<I2cDeviceSynch> implements ColorSensor {

    public static final I2cAddr I2CADDR_DEFAULT = I2cAddr.create7bit(0x29);

    public static final int TCS34725_COMMAND_BIT = 0x80;

    @SuppressWarnings("unused")
    public class Registers {
        public static final int TCS34725_ENABLE = 0x00;
        public static final int TCS34725_ENABLE_AIEN = 0x10;
        public static final int TCS34725_ENABLE_WEN = 0x08;
        public static final int TCS34725_ENABLE_AEN = 0x02;
        public static final int TCS34725_ENABLE_PON = 0x01;
        public static final int TCS34725_ATIME = 0x01;
        public static final int TCS34725_WTIME = 0x03;
        public static final int TCS34725_WTIME_2_4MS = 0xFF;
        public static final int TCS34725_WTIME_204MS = 0xAB;
        public static final int TCS34725_WTIME_614MS = 0x00;
        public static final int TCS34725_AILTL = 0x04;
        public static final int TCS34725_AILTH = 0x05;
        public static final int TCS34725_AIHTL = 0x06;
        public static final int TCS34725_AIHTH = 0x07;
        public static final int TCS34725_PERS = 0x0C;
        public static final int TCS34725_PERS_NONE = 0b0000;
        public static final int TCS34725_PERS_1_CYCLE = 0b0001;
        public static final int TCS34725_PERS_2_CYCLE = 0b0010;
        public static final int TCS34725_PERS_3_CYCLE = 0b0011;
        public static final int TCS34725_PERS_5_CYCLE = 0b0100;
        public static final int TCS34725_PERS_10_CYCLE = 0b0101;
        public static final int TCS34725_PERS_15_CYCLE = 0b0110;
        public static final int TCS34725_PERS_20_CYCLE = 0b0111;
        public static final int TCS34725_PERS_25_CYCLE = 0b1000;
        public static final int TCS34725_PERS_30_CYCLE = 0b1001;
        public static final int TCS34725_PERS_35_CYCLE = 0b1010;
        public static final int TCS34725_PERS_40_CYCLE = 0b1011;
        public static final int TCS34725_PERS_45_CYCLE = 0b1100;
        public static final int TCS34725_PERS_50_CYCLE = 0b1101;
        public static final int TCS34725_PERS_55_CYCLE = 0b1110;
        public static final int TCS34725_PERS_60_CYCLE = 0b1111;
        public static final int TCS34725_CONFIG = 0x0D;
        public static final int TCS34725_CONFIG_WLONG = 0x02;
        public static final int TCS34725_CONTROL = 0x0F;
        public static final int TCS34725_ID = 0x12;
        public static final int TCS34725_STATUS = 0x13;
        public static final int TCS34725_STATUS_AINT = 0x10;
        public static final int TCS34725_STATUS_AVALID = 0x01;
        public static final int TCS34725_CDATAL = 0x14;
        public static final int TCS34725_CDATAH = 0x15;
        public static final int TCS34725_RDATAL = 0x16;
        public static final int TCS34725_RDATAH = 0x17;
        public static final int TCS34725_GDATAL = 0x18;
        public static final int TCS34725_GDATAH = 0x19;
        public static final int TCS34725_BDATAL = 0x1A;
        public static final int TCS34725_BDATAH = 0x1B;
    }

    public enum IntegrationTime {
        INTEGRATION_TIME_2_4MS(0xFF),
        INTEGRATION_TIME_24MS(0xF6),
        INTEGRATION_TIME_50MS(0xEB),
        INTEGRATION_TIME_101MS(0xD5),
        INTEGRATION_TIME_154MS(0xC0),
        INTEGRATION_TIME_700MS(0x00);
        public final byte byteVal;

        IntegrationTime(int val) {
            byteVal = (byte) val;
        }
    }

    public enum Gain {
        GAIN_1X(0x00),
        GAIN_4X(0x01),
        GAIN_16X(0x02),
        GAIN_60X(0x03);
        public final byte byteVal;

        Gain(int val) {
            byteVal = (byte) val;
        }
    }

    private static final I2cDeviceSynch.ReadWindow READ_WINDOW = new I2cDeviceSynch.ReadWindow(Registers.TCS34725_CDATAL | TCS34725_COMMAND_BIT, 8, I2cDeviceSynch.ReadMode.REPEAT);

    private IntegrationTime integrationTime;
    private Gain gain;
    private I2cAddr i2cAddr;
    private DigitalChannel ledChannel;

    public TCS34725ColorSensor(I2cDeviceSynch deviceClient, boolean isOwned) {
        this(I2CADDR_DEFAULT, deviceClient, null, isOwned);
    }

    public TCS34725ColorSensor(I2cAddr addr, I2cDeviceSynch deviceClient, DigitalChannel digitalChannel, boolean isOwned) {
        super(deviceClient, isOwned);
        ledChannel = digitalChannel;
        if (ledChannel != null) ledChannel.setMode(DigitalChannelController.Mode.OUTPUT);
        i2cAddr = addr;
        gain = Gain.GAIN_1X;
        integrationTime = IntegrationTime.INTEGRATION_TIME_700MS;

        deviceClient.setLoggingTag("TCS34725");
        deviceClient.setLogging(true);
    }

    public void write8(int reg, int data) {
        deviceClient.write8(TCS34725_COMMAND_BIT | reg, data);
        deviceClient.waitForWriteCompletions(I2cWaitControl.WRITTEN);
    }

    public byte[] read(int reg, int creg) {
        return deviceClient.read(TCS34725_COMMAND_BIT | reg, creg);
    }

    public int read8(int reg) {
        return deviceClient.read8(TCS34725_COMMAND_BIT | reg) & 0xFF;
    }

    public int read16(int reg) {
        byte[] data = deviceClient.read(TCS34725_COMMAND_BIT | reg, 2);
        return TypeConversion.byteArrayToShort(data, ByteOrder.LITTLE_ENDIAN);
    }

    public int make16(byte lower, byte upper) {
        return TypeConversion.byteArrayToShort(new byte[]{lower, upper}, ByteOrder.LITTLE_ENDIAN);
    }

    private void delay(int ms) {
        try {
            Thread.sleep(ms);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    @Override
    protected boolean doInitialize() {
        this.deviceClient.engage();

        this.deviceClient.setI2cAddress(i2cAddr);

        int deviceId = read8(Registers.TCS34725_ID);
        if ((deviceId != 0x44) && (deviceId != 0x10)) {
            return false;
        }

        write8(Registers.TCS34725_ATIME, integrationTime.byteVal);
        write8(Registers.TCS34725_CONTROL, gain.byteVal);

        enable();

        deviceClient.setReadWindow(READ_WINDOW);

        return true;
    }

    public void enable() {
        write8(Registers.TCS34725_ENABLE, Registers.TCS34725_ENABLE_PON);
        delay(3);
        write8(Registers.TCS34725_ENABLE, Registers.TCS34725_ENABLE_PON | Registers.TCS34725_ENABLE_AEN);
    }

    public void disable() {
        int data = read8(Registers.TCS34725_ENABLE);
        write8(Registers.TCS34725_ENABLE, data & ~(Registers.TCS34725_ENABLE_PON | Registers.TCS34725_ENABLE_AEN));
    }

    public void setIntegrationTime(IntegrationTime time) {
        if (isInitialized) write8(Registers.TCS34725_ATIME, time.byteVal);
        integrationTime = time;
    }

    public IntegrationTime getIntegrationTime() {
        return integrationTime;
    }

    public void setGain(Gain gain) {
        if (isInitialized) write8(Registers.TCS34725_CONTROL, gain.byteVal);
        this.gain = gain;
    }

    public Gain getGain() {
        return gain;
    }

    @Override
    public int red() {
        return read16(Registers.TCS34725_RDATAL);
    }

    @Override
    public int green() {
        return read16(Registers.TCS34725_GDATAL);
    }

    @Override
    public int blue() {
        return read16(Registers.TCS34725_BDATAL);
    }

    @Override
    public int alpha() {
        return read16(Registers.TCS34725_CDATAL);
    }

    @Override
    public int argb() {
        throw new UnsupportedOperationException("This method is not supported");
    }

    @Override
    public void enableLed(boolean enable) {
        if (ledChannel != null) ledChannel.setState(enable);
    }

    @Override
    public void setI2cAddress(I2cAddr newAddress) {
        throw new UnsupportedOperationException("The TCS34725 I2C address is unmodifiable");
    }

    @Override
    public I2cAddr getI2cAddress() {
        return deviceClient.getI2cAddress();
    }

    @Override
    public Manufacturer getManufacturer() {
        return Manufacturer.Adafruit;
    }

    @Override
    public String getDeviceName() {
        return "AdaFruit TCS34725 Color Sensor";
    }

    @Override
    public String getConnectionInfo() {
        return deviceClient.getConnectionInfo();
    }

    @Override
    public int getVersion() {
        return 0;
    }

    @Override
    public void resetDeviceConfigurationForOpMode() {

    }

    @Override
    public void close() {
        if (this.deviceClientIsOwned) {
            this.deviceClient.close();
        }
    }
}
