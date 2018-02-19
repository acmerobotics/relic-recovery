package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SideDistanceTest extends OpMode {
    private Robot robot;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.drive.extendSideSwivel();
        robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);
        robot.start();
    }

    @Override
    public void loop() {
        telemetry.addData("sideDistance", robot.drive.getSideDistance(DistanceUnit.INCH));
    }
}
