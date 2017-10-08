package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kelly on 9/28/2017.
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
 */
public class MecanumDrive {

    /***
     * K = (wheelbsae width + wheelbase height) / 4
     */
    public static double K = (18 + 18) / 4;

    private DcMotor[] motors;
    private int[] offsets;

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
        for (int i = 0; i < 4; i ++) {
            motors[i] = map.get(DcMotor.class, names[i]);
        }
        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);
        resetEncoders();
    }

    /**
     * not as pretty but the exactly same result as the other version
     * [V] = [vX, vY, vOmega]'
     * [R] = [1, -1, -K]
     *       [1,  1, -K]
     *       [1, -1,  K]
     *       [1,  1,  K]
     *
     * [F] = [V][R]
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
