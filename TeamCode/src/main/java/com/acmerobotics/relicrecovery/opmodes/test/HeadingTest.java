package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class HeadingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDashboard dashboard = RobotDashboard.getInstance();

        LynxModule lynxModule = hardwareMap.get(LynxModule.class, "rearHub");
        BNO055IMU imu = new AdafruitBNO055IMU(LynxOptimizedI2cSensorFactory.createLynxI2cDeviceSync(lynxModule, 1));
//        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        while (opModeIsActive()) {
            dashboard.getTelemetry().addData("heading", imu.getAngularOrientation().firstAngle);
            dashboard.getTelemetry().update();
        }
    }
}
