package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp
public class MotorSpeedTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotor[] motors = new DcMotor[4];
        ExponentialSmoother[] smoothers = new ExponentialSmoother[4];
        for (int i = 0; i < 4; i++) {
            motors[i] = hardwareMap.dcMotor.get(MecanumDrive.MOTOR_NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            smoothers[i] = new ExponentialSmoother(0.001);
        }

        waitForStart();

        int[] lastPositions = new int[4];
        double[] lastTimestamps = new double[4];

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(0.5);
        }

        while (!isStopRequested()) {
            for (int i = 0; i < 4; i++) {
                int position = motors[i].getCurrentPosition();
                double timestamp = TimestampedData.getCurrentTime();
                if (lastTimestamps[i] != 0) {
                    double dt = timestamp - lastTimestamps[i];
                    double speed = (position - lastPositions[i]) / dt;
                    double smoothedSpeed = smoothers[i].update(speed);
                    telemetry.addData(MecanumDrive.MOTOR_NAMES[i] + "Speed", (int) smoothedSpeed);
                }
                lastPositions[i] = position;
                lastTimestamps[i] = timestamp;
            }
            telemetry.update();
        }
    }
}
