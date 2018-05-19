package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.dashboard.RobotDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class HeadingTest2 extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDashboard dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        MecanumDrive drive = new MecanumDrive(hardwareMap);
        drive.setEstimatedPose(new Pose2d(0, 0, Math.PI));
        drive.enablePositionEstimation();

        waitForStart();

        while (opModeIsActive()) {
            Pose2d estimatedPose = drive.getEstimatedPose();
            telemetry.addData("heading", estimatedPose.heading());
            telemetry.update();
            drive.update(null);
        }
    }
}
