package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.hardware.bosch.BNO055IMU;

import org.firstinspires.ftc.robotcore.external.navigation.Acceleration;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.nio.ByteBuffer;
import java.nio.ByteOrder;

public class BatchingBNO055IMU {
    private BNO055IMU imu;

    public static class BulkData {
        public final Orientation orientation;
        public final Acceleration linearAcceleration;

        private BulkData(Orientation orientation, Acceleration acceleration) {
            this.orientation = orientation;
            this.linearAcceleration = acceleration;
        }
    }

    public BatchingBNO055IMU(BNO055IMU imu) {
        this.imu = imu;
    }

    public BulkData getBulkData() {
        byte[] rawData = imu.read(BNO055IMU.Register.EUL_H_LSB, 6 + 8 + 6);
        // TODO: we fake the timestamp since readTimestamped() is not easily accessible
        // probably not a big deal but we might want to reconsider this down the line
        long timestamp = System.nanoTime();

        // scales; stolen from BNO055IMUImpl
        BNO055IMU.Parameters parameters = imu.getParameters();
        float angularScale = parameters.angleUnit == BNO055IMU.AngleUnit.DEGREES ? 16.0f : 900.0f;
        float accelerationScale = 100.0f;

        // extract the values
        ByteBuffer buffer = ByteBuffer.wrap(rawData).order(ByteOrder.LITTLE_ENDIAN);
        float heading = buffer.getShort() / angularScale;
        float roll = buffer.getShort() / angularScale;
        float pitch = buffer.getShort() / angularScale;
        buffer.position(6 + 8);
        float accelerationX = buffer.getShort() / accelerationScale;
        float accelerationY = buffer.getShort() / accelerationScale;
        float accelerationZ = buffer.getShort() / accelerationScale;

        AngleUnit angleUnit = parameters.angleUnit == BNO055IMU.AngleUnit.DEGREES ? AngleUnit.DEGREES : AngleUnit.RADIANS;
        Orientation orientation = new Orientation(AxesReference.INTRINSIC, AxesOrder.ZYX, angleUnit,
                angleUnit.normalize(-heading),
                angleUnit.normalize(roll),
                angleUnit.normalize(pitch),
                timestamp);
        Acceleration acceleration = new Acceleration(DistanceUnit.METER, accelerationX, accelerationY, accelerationZ, timestamp);
        return new BulkData(orientation, acceleration);
    }
}
