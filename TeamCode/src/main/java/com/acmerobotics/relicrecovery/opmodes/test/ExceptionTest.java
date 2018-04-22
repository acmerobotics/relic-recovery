package com.acmerobotics.relicrecovery.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

@Disabled
@Autonomous
public class ExceptionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        waitForStart();

        AppUtil.getInstance().runOnUiThread(() -> {
            throw new RuntimeException("Exception test");
        });
    }
}
