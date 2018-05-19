package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

@Autonomous
public class UltrasonicSamplingTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        CSVLoggingTelemetry distanceLog = new CSVLoggingTelemetry(
                new File(LoggingUtil.getLogRoot(this), "UltrasonicSamples-" + System.currentTimeMillis() + ".csv"));
        Robot robot = new Robot(this);
        robot.drive.enablePositionEstimation();
        robot.drive.setEstimatedPosition(new Vector2d(47, 0));
        robot.start();

        waitForStart();

        robot.drive.extendUltrasonicSwivel();

        sleep(500);

        robot.drive.setVelocity(new Vector2d(-0.02, 0), 0);
        robot.drive.enableHeadingCorrection();

        while (opModeIsActive()) {
            distanceLog.addData("timestamp", TimestampedData.getCurrentTime());
            distanceLog.addData("ultrasonicDistance", robot.drive.getUltrasonicDistance(DistanceUnit.INCH));
            distanceLog.addData("x", robot.drive.getEstimatedPosition().x());
            distanceLog.update();

            robot.waitForNextCycle();
        }
    }
}
