package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

import java.util.Arrays;
import java.util.Collections;

/**
 * Created by kelly on 9/28/2017
 *
 * Wheel layout (top view):
 *
 *        FRONT
 * (1)\\---------//(4)
 *      |       |
 *      |       |
 *      |       |
 *      |       |
 * (2)//---------\\(3)
 *
 * the paper: http://www.chiefdelphi.com/media/papers/download/2722 (see doc/Mecanum_Kinematic_Analysis_100531.pdf)
 */
public class MecanumDrive {
    // TODO: should these be extracted into some kind of configuration object?
    public static final double WHEELBASE_WIDTH = 18;
    public static final double WHEELBASE_HEIGHT = 18;

    /***
     * K = (wheelbase width + wheelbase height) / 4 (in)
     */
    public static final double K = (WHEELBASE_WIDTH + WHEELBASE_HEIGHT) / 4;

    /**
     * wheel radius (in)
     */
    public static final double RADIUS = 2;

    private DcMotor[] motors;

    /**
     * units in encoder ticks; solely intended for internal use
     */
    private int[] offsets;

    private BNO055IMU imu;
    private double headingOffset;

    /**
     * construct drive with default configuration names
     * @param map hardware map
     */
    public MecanumDrive(HardwareMap map) {
        this(map, new String[]{"frontLeft", "rearLeft", "rearRight", "frontRight"});
    }

    /**
     * construct drive with configuration names other than the default
     * @param map hardware map
     * @param names names of the motors in the hardware mapping
     */
    public MecanumDrive(HardwareMap map, String[] names) {
        if (names.length != 4) {
            throw new IllegalArgumentException("must be four for motors");
        }

        imu = map.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        resetHeading();

        offsets = new int[4];
        motors = new DcMotor[4];
        for (int i = 0; i < 4; i ++) {
            motors[i] = map.dcMotor.get(names[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        resetEncoders();
    }

    /**
     * not as pretty but the exactly same result as the other version
     * [W] is wheel rotations
     * [V] = [vX, vY, vOmega]'
     * [R] = [1, -1, -K]
     *       [1,  1, -K]
     *       [1, -1,  K]
     *       [1,  1,  K]
     *
     * [W] = [V][R]
     *
     * @param vel
     * @param omega
     */
    // TODO: do these equations hold for a non-square wheelbase?
    public void setVelocity(Vector2d vel, double omega) {
        double[] power = new double[4];
        power[0] = vel.x() - vel.y() - K * omega;
        power[1] = vel.x() + vel.y() - K * omega;
        power[2] = vel.x() - vel.y() + K * omega;
        power[3] = vel.x() + vel.y() + K * omega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(power[0]),
			Math.abs(power[1]), Math.abs(power[2]), Math.abs(power[3])));

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(power[i] / max);
        }
    }

    /**
     * get distance traveled from encoder values
     *
     * [W] is wheel rotations
     * [V] = [vX, vY, vOmega]'
     * [F] = [ 1/4,    1/4,   1/4,   1/4]
     *       [-1/4,    1/4,  -1/4,   1/4]
     *       [-1/4K, -1/4K,  1/4K,  1/4K]
     *
     * [V] = (r)[W][F]
     *
     * @param rot rotation of each wheel, in radians
     * @return movement of robot
     */
    public Pose2d getPoseDelta(double[] rot) {
        if (rot.length != 4) {
            throw new IllegalArgumentException("length must be four");
        }
        double x = RADIUS * ( rot[0] + rot[1] + rot[2] + rot[3]) / 4;
        double y = RADIUS * (-rot[0] + rot[1] - rot[2] + rot[3]) / 4;
        double h = RADIUS * (-rot[0] - rot[1] + rot[2] + rot[3]) / (4 * MecanumDrive.K);
        return new Pose2d(x, y, h);
    }

    public void resetEncoders() {
        for(int i = 0; i < 4; i++) {
            offsets[i] = -motors[i].getCurrentPosition();
        }
    }

    /** @return motor rotations in radians */
    public double[] getRotations() {
        double[] rotations = new double[4];
        for (int i = 0; i < 4; i++) {
            rotations[i] = getRotation(i);
        }
        return rotations;
    }

    /** @return motor rotation in radians */
    public double getRotation(int motor) {
        return ticksToRadians(motor, getPosition(motor));
    }

    /** @return motor positions in encoder ticks */
    public int[] getPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = getPosition(i);
        }
        return positions;
    }

    /** @return motor position in encoder ticks */
    public int getPosition(int motor) {
        return offsets[motor] + motors[motor].getCurrentPosition();
    }

    private double ticksToRadians(int motor, int ticks) {
        double ticksPerRev = motors[motor].getMotorType().getTicksPerRev();
        return 2 * Math.PI * ticks / ticksPerRev;
    }

    private double getRawHeading() {
        return imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
    }

    public double getHeading() {
        return getRawHeading() + headingOffset;
    }

    public void resetHeading() {
        headingOffset = -getRawHeading();
    }

    // TODO: stub
    public void turn(double angle) {

    }

    // TODO: stub
    public void move(double distance, double speed) {

    }
}
