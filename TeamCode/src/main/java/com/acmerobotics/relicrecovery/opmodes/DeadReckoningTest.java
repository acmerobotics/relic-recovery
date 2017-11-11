package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author kellyrm
 */

// TODO: refactor; combine w/ PoseEstimationTest?
@TeleOp(name="Dead Reckoning Test")
public class DeadReckoningTest extends OpMode{

    private MecanumDrive drive;
    private BNO055IMU imu;
    private Pose2d position;
    private double[] lastRotations;

    public void init() {
        position = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap);
        lastRotations = drive.getRotations();
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);
    }

    public void loop() {
        double theta = imu.getAngularOrientation().firstAngle;
        double forward = -gamepad1.left_stick_x;
        double right = -gamepad1.left_stick_y;
        double omega = -gamepad1.right_stick_x;
        Vector2d vel = new Vector2d(forward * Math.cos(theta) - right * Math.sin(theta), forward * Math.sin(theta) + right * Math.cos(theta));
        drive.setVelocity(vel, omega);

        if (gamepad1.a) {
            position = new Pose2d (0,0,0);
        }

        double[] rotations = drive.getRotations();
        double[] rotationsDelta = new double[4];
        for (int i = 0; i < 4; i++) {
            rotationsDelta[i] = rotations[i] - lastRotations[i];
        }
        position.add(drive.getPoseDelta(rotationsDelta));

        telemetry.addData("x", position.x());
        telemetry.addData("y", position.y());
        telemetry.addData("heading", position.heading());
    }

}
