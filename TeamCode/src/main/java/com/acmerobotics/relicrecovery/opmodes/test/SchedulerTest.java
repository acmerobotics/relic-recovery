package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ryanbrott on 1/7/18.
 */

@TeleOp(name = "Scheduler Test")
public class SchedulerTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        PriorityScheduler scheduler = new PriorityScheduler();
        Looper looper = new Looper(scheduler, Looper.DEFAULT_LOOP_TIME);
        BNO055IMU imu = LynxOptimizedI2cSensorFactory.createLynxEmbeddedIMU(hardwareMap.getAll(LynxModule.class).iterator().next());
        imu.initialize(new BNO055IMU.Parameters());
        DcMotor motor = hardwareMap.getAll(DcMotor.class).iterator().next();
        looper.addLoop(((timestamp, dt) ->
            scheduler.add(() -> motor.setPower(TimestampedData.getCurrentTime() % 1), "motor set power", PriorityScheduler.HIGH_PRIORITY)
        ));
        scheduler.addRepeating(imu::getAngularOrientation, "imu read", PriorityScheduler.LOW_PRIORITY);
        scheduler.addRepeating(motor::getCurrentPosition, "encoder read", PriorityScheduler.LOW_PRIORITY);

        waitForStart();

        looper.start();
        scheduler.start();

        while (opModeIsActive());
    }
}
