package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.Locale;

/**
 * Created by ryanbrott on 10/28/17.
 */

@TeleOp(name = "Pose Estimation Test")
public class PoseEstimationTest extends OpMode {
    private RobotDashboard dashboard;
    private Canvas fieldOverlay;

    private BNO055IMU imu;

    private MecanumDrive drive;
    private int[] lastPositions;
    private Pose2d pose;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        fieldOverlay = dashboard.getFieldOverlay();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        drive = new MecanumDrive(hardwareMap);

        pose = new Pose2d(0, 0, 0);
    }

    @Override
    public void loop() {
        if (lastPositions == null) {
            lastPositions = drive.getPositions();
        } else {
            int[] positions = drive.getPositions();
            int[] positionDeltas = new int[positions.length];
            for (int i = 0; i < positions.length; i++) {
                positionDeltas[i] = positions[i] - lastPositions[i];
            }

            Pose2d poseDelta = drive.getDelta(positionDeltas);

            pose.add(poseDelta);

            lastPositions = positions;
        }

        if (gamepad1.a) {
            pose = new Pose2d(0, 0, 0);
        }

        drive.setVelocity(new Vector2d(
                -gamepad1.left_stick_y,
                -gamepad1.left_stick_x
        ), -gamepad1.right_stick_y);

        double actualHeading = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;

        fieldOverlay.setFill("green");
        fieldOverlay.fillCircle(pose.x(), pose.y(), 5);
        dashboard.drawOverlay();

        telemetry.addData(">", "Press [A] to reset pose to (0, 0, 0)");
        telemetry.addData("x", pose.x());
        telemetry.addData("y", pose.y());
        telemetry.addData("estimatedHeading", String.format(Locale.ENGLISH, "%.2f (%.2fdeg)", pose.heading(), Math.toDegrees(pose.heading())));
        int[] pos = drive.getPositions();
        for (int i = 0; i < pos.length; i++) {
            telemetry.addData("encoder" + i, pos[i]);
        }
        telemetry.addData("actualHeading", actualHeading);

        try {
            Thread.sleep(25);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
