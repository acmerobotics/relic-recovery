package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class AutoIntakeTest extends OpMode {
    @Override
    public void init() {
        Robot robot = new Robot(this);
        robot.intake.autoIntake();
        robot.start();
    }

    @Override
    public void loop() {

    }
}
