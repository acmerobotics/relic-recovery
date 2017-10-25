package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

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
 * the paper: http://www.chiefdelphi.com/media/papers/download/2722
 */
public class MecanumDrive {

    /***
     * K = (wheelbase width + wheelbase height) / 4 (in)
     */
    public static double K = (18 + 18) / 4;

    /**
     * wheel radius (in)
     */
    public static double RADIUS = 2;

    private DcMotor[] motors;
    private int[] offsets;

    public enum Side {
        LEFT,
        RIGHT;
    }

    /**
     * construct drive with default configuration names
     * @param map hardware map
     */
    public MecanumDrive(HardwareMap map) {
        this(map, new String[]{"frontLeft", "rearLeft", "rearRight", "frontRight"}, Side.LEFT);
    }

    /**
     * construct drive with configuration names other than the default
     * @param map hardware map
     * @param names names of the motors in the hardware mapping
     */
    public MecanumDrive(HardwareMap map, String[] names, Side reversed) {
        if (names.length != 4) {
            throw new IllegalArgumentException("must be four for motors");
        }
        offsets = new int[4];
        motors = new DcMotor[4];
        for (int i = 0; i < 4; i ++) {
            motors[i] = map.dcMotor.get(names[i]);
        }

        if (reversed == Side.LEFT) {
            motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
            motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        } else {
            motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
            motors[3].setDirection(DcMotorSimple.Direction.REVERSE);
        }

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
    public void setVelocity(Vector2d vel, double omega) {
        double[] power = new double[4];
        power[0] = vel.x() - vel.y() - K * omega;
        power[1] = vel.x() + vel.y() - K * omega;
        power[2] = vel.x() - vel.y() + K * omega;
        power[3] = vel.x() + vel.y() + K * omega;
        for (int i = 0; i < 4; i++) {
            motors[i].setPower(power[i]);
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
    public static Pose2d getDelta(int[] rot) {
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

    public int[] getPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = offsets[i] + motors[i].getCurrentPosition();
        }
        return positions;
    }

    public int getPosition(int motor) {
        return getPositions()[motor];
    }

}
