package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.relicrecovery.opmodes.StickyGamepad;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;

import java.io.File;

@TeleOp
public class GlyphColorRecorder extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        robot.start();

        StickyGamepad stickyGamepad1 = new StickyGamepad(gamepad1);

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(new File(
                LoggingUtil.getLogRoot(this), "GlyphColorResults-" + System.currentTimeMillis() + ".csv"));

        telemetry.log().add("Ready! Press play to begin.");
        telemetry.update();

        waitForStart();

        int samplesCollected = 0;

        while (opModeIsActive()) {
            stickyGamepad1.update();

            if (stickyGamepad1.a || stickyGamepad1.b) {
                NormalizedRGBA color = robot.intake.getFrontNormalizedColors();
                loggingTelemetry.addData("color", stickyGamepad1.a ? "gray" : "brown");
                loggingTelemetry.addData("red", color.red);
                loggingTelemetry.addData("green", color.green);
                loggingTelemetry.addData("blue", color.blue);
                loggingTelemetry.update();
                samplesCollected++;
            }

            if (stickyGamepad1.x || stickyGamepad1.y) {
                NormalizedRGBA color = robot.intake.getRearNormalizedColors();
                loggingTelemetry.addData("color", stickyGamepad1.x ? "gray" : "brown");
                loggingTelemetry.addData("red", color.red);
                loggingTelemetry.addData("green", color.green);
                loggingTelemetry.addData("blue", color.blue);
                loggingTelemetry.update();
                samplesCollected++;
            }

            telemetry.log().clear();
            telemetry.log().add("Press the right button to record a new sample:");
            telemetry.log().add("[A] front gray glyph");
            telemetry.log().add("[B] front brown glyph");
            telemetry.log().add("[X] rear gray glyph");
            telemetry.log().add("[Y] rear brown glyph");
            telemetry.log().add(samplesCollected + " samples collected.");
            telemetry.update();
        }
    }
}
