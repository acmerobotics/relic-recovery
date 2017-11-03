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

@TeleOp(name="Dead Reckoning Test")
public class DeadReckoningTest extends OpMode{

    public static final double TICS_PER_REV = 20;

    private MecanumDrive drive;
    private BNO055IMU imu;
    private Pose2d position;
    private int[] encoders;

    public void init() {
        position = new Pose2d(0, 0, 0);
        drive = new MecanumDrive(hardwareMap);
        encoders = drive.getPositions();
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

        int[] encodersNew = drive.getPositions();
        double[] rotations = new double[4];
        for (int i = 0; i < 4; i++) {
            rotations[i] = (encodersNew[i] - encoders[i]) / TICS_PER_REV;
        }
        position.add(drive.getDelta(rotations));

        telemetry.addData("x", position.x());
        telemetry.addData("y", position.y());
        telemetry.addData("heading", position.heading());
    }

}