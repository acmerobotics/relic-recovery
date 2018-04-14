package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorEx;

@TeleOp
public class EncoderTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx[] dcMotors = new DcMotorEx[4];
        for (int i = 0; i < 4; i++) {
            dcMotors[i] = hardwareMap.get(DcMotorEx.class, MecanumDrive.MOTOR_NAMES[i]);
            dcMotors[i].setMotorDisable();
        }

        waitForStart();

        while (opModeIsActive()) {
            for (int i = 0; i < 4; i++) {
                telemetry.addData("encoder" + i, dcMotors[i].getCurrentPosition());
            }
            telemetry.update();
        }
    }
}
