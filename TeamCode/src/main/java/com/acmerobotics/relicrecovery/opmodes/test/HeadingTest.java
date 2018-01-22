package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

@TeleOp
public class HeadingTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDashboard dashboard = RobotDashboard.getInstance();

//        LynxModule lynxModule = hardwareMap.get(LynxModule.class, "rearHub");
//        BNO055IMU imu = LynxOptimizedI2cSensorFactory.createLynxBNO055IMU(lynxModule, 1);
        BNO055IMU imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(new BNO055IMU.Parameters());

        while (opModeIsActive()) {
            dashboard.getTelemetry().addData("heading", imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle);
            dashboard.getTelemetry().update();
        }
    }
}
