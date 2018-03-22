package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.hardware.BatchingBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

@Autonomous
public class BatchingIMUTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(RobotDashboard.getInstance().getTelemetry(), telemetry);

        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        imu.initialize(parameters);
        BatchingBNO055IMU batchingImu = new BatchingBNO055IMU(imu);

        waitForStart();

        double sum = 0;
        int i = 0;
        while (opModeIsActive()) {
            double timestamp = TimestampedData.getCurrentTime();
            BatchingBNO055IMU.BulkData bulkData = batchingImu.getBulkData();
            double elapsedTime = TimestampedData.getCurrentTime() - timestamp;
            if (i < 100) {
                sum += elapsedTime;
                i++;
            } else {
                sum = 0;
                i = 0;
                RobotLog.i("Avg read time: " + sum / 100);
            }

            telemetry.addData("heading", bulkData.orientation.firstAngle);
            telemetry.addData("pitch", bulkData.orientation.secondAngle);
            telemetry.addData("roll", bulkData.orientation.thirdAngle);

            telemetry.addData("accelerationX", bulkData.linearAcceleration.xAccel);
            telemetry.addData("accelerationY", bulkData.linearAcceleration.yAccel);
            telemetry.addData("accelerationZ", bulkData.linearAcceleration.zAccel);

            telemetry.update();
        }
    }
}
