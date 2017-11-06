package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * Created by ryanbrott on 11/5/17.
 */

@TeleOp
public class MainTeleOp extends OpMode {
    private MecanumDrive drive;
    private boolean fieldCentric, halfSpeed;
    private StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());
        drive = new MecanumDrive(hardwareMap);
        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        stickyGamepad1.update();

        if (stickyGamepad1.a) {
            fieldCentric = !fieldCentric;
        }

        if (stickyGamepad1.y) {
            halfSpeed = !halfSpeed;
        }

        double x = -gamepad1.left_stick_y;
        double y = -gamepad1.left_stick_x;
        double omega = -gamepad1.right_stick_x;
        double heading = drive.getHeading();

        if (halfSpeed) {
            x *= 0.5;
            y *= 0.5;
            omega *= 0.5;
        }

        if (fieldCentric) {
            drive.setVelocity(new Vector2d(x * Math.cos(heading) - y * Math.sin(heading), x * Math.sin(heading) + y * Math.cos(heading)), omega);
        } else {
            drive.setVelocity(new Vector2d(x, y), omega);
        }

        telemetry.addData(">", fieldCentric ? "Field Centric (A to switch)" : "Robot Centric (A to switch)");
        telemetry.addData("x", x);
        telemetry.addData("y", y);
        telemetry.addData("omega", omega);
        telemetry.addData("heading", heading);
    }
}
