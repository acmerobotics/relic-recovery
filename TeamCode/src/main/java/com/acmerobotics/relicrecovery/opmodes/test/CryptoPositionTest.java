package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.util.DrawingUtil;
import com.acmerobotics.relicrecovery.vision.CryptoboxLocalizer;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp
public class CryptoPositionTest extends LinearOpMode {
    @Override
    public void runOpMode() throws InterruptedException {
        RobotDashboard dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        VuforiaCamera vuforiaCamera = new VuforiaCamera();
        CryptoboxTracker cryptoboxTracker = new CryptoboxTracker(AllianceColor.BLUE);
        CryptoboxLocalizer localizer = new CryptoboxLocalizer(cryptoboxTracker,
                vuforiaCamera.getProperties(), new MecanumDrive(hardwareMap, telemetry));
        localizer.addListener(new CryptoboxLocalizer.Listener() {
            @Override
            public void onPositionUpdate(Vector2d position, double timestamp) {
                DrawingUtil.drawMecanumRobot(dashboard.getFieldOverlay(), new Pose2d(position, -Math.PI / 2));
                telemetry.addData("x", position.x());
                telemetry.addData("y", position.y());
                telemetry.update();
            }
        });

        waitForStart();

        while (opModeIsActive());
    }
}
