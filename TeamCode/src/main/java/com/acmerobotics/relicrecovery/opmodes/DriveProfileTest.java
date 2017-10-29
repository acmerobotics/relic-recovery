package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

/**
 * Created by ryanbrott on 10/28/17.
 */

@Autonomous(name = "Drive Profile Test")
public class DriveProfileTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        MecanumDrive drive = new MecanumDrive(hardwareMap);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData(">", "Press [A] to drive a distance of 3 tiles");
            telemetry.update();

            while (opModeIsActive() && !gamepad1.a) { }

            drive.driveSync(72);
        }
    }
}
