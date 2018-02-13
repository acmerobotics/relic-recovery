package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.relicrecovery.util.DrawingUtil;
import com.acmerobotics.relicrecovery.vision.CryptoboxLocalizer;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class CryptoPositionTest extends LinearOpMode {
    private Vector2d visionPosition;

    @Override
    public void runOpMode() throws InterruptedException {
        Robot robot = new Robot(this);
        robot.addListener(() -> {
            if (visionPosition != null) {
                Canvas canvas = robot.dashboard.getFieldOverlay();
                canvas.setStroke("yellow");
                DrawingUtil.drawMecanumRobot(canvas, new Pose2d(visionPosition, Math.PI / 2));
                telemetry.addData("visionX", visionPosition.x());
                telemetry.addData("visionY", visionPosition.y());
            }
        });
        robot.start();
        robot.phoneSwivel.pointAtCryptobox();

        telemetry = new MultipleTelemetry(telemetry, robot.dashboard.getTelemetry());

        VuforiaCamera vuforiaCamera = new VuforiaCamera();
        CryptoboxTracker cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        CryptoboxLocalizer localizer = new CryptoboxLocalizer(cryptoboxTracker,
                vuforiaCamera.getProperties(), new MecanumDrive(hardwareMap, telemetry));
        localizer.addListener((position, timestamp) -> visionPosition = position);
        vuforiaCamera.initialize();

        waitForStart();

        while (opModeIsActive());
    }
}
