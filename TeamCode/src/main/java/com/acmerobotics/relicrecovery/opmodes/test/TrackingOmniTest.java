package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.localization.TrackingOmniLocalizer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class TrackingOmniTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        Robot robot = new Robot(this);
        robot.drive.setLocalizer(new TrackingOmniLocalizer(robot.drive));
        robot.drive.enablePositionEstimation();
        robot.start();

        waitForStart();

        while (opModeIsActive());
    }
}
