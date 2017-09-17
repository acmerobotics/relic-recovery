package com.acmerobotics.velocityvortex.drive;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.Range;

/**
 * This class implements the basic functionality of an omnidirectional mecanum wheel drive system.
 */
public class MecanumDrive {

    public static final String[] MOTOR_ORDER = {
            "leftFront",
            "rightFront",
            "leftBack",
            "rightBack"
    };

    public static final Vector2d[] ROLLER_DIRS = {
            new Vector2d(-1, 1).normalize(),
            new Vector2d(1, 1).normalize(),
            new Vector2d(1, 1).normalize(),
            new Vector2d(-1, 1).normalize()
    };

    public static final Vector2d[] ROTATION_DIRS = {
            new Vector2d(0, -1),
            new Vector2d(0, 1),
            new Vector2d(0, -1),
            new Vector2d(0, 1)
    };

    private DcMotor[] motors;
    private double maxAchievableRpm;
    private DcMotor.RunMode runMode;

    public MecanumDrive(HardwareMap map) {
        motors = new DcMotor[4];
        for (int i = 0; i < motors.length; i++) {
            motors[i] = map.dcMotor.get(MOTOR_ORDER[i]);
        }

        maxAchievableRpm = Double.MAX_VALUE;
        for (DcMotor motor : motors) {
            MotorConfigurationType type = motor.getMotorType();
            double achievableRpm = type.getMaxRPM() * type.getAchieveableMaxRPMFraction();
            if (achievableRpm < maxAchievableRpm) {
                maxAchievableRpm = achievableRpm;
            }
        }

        setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public DcMotor[] getMotors() {
        return motors;
    }

    /**
     * Sets the velocity of the mecanum drive system.
     *
     * @param v translational velocity
     * @see #setVelocity(Vector2d, double)
     */
    public void setVelocity(Vector2d v) {
        setVelocity(v, 0);
    }

    /**
     * Sets the velocity of the mecanum drive system. This includes both the translational
     * component and the angular component. A positive speed indicates a clockwise rotation, and a
     * negative speed indicates a counter-clockwise rotation.
     *
     * @param v            translational velocity
     * @param angularSpeed angular speed
     */
    public void setVelocity(Vector2d v, double angularSpeed) {
        angularSpeed = Range.clip(angularSpeed, -1, 1);
        double speed;
        if (v.norm() > 1) {
            speed = 1;
        } else {
            speed = v.norm();
        }

        if (Math.abs(speed) > 1E-10) {
            v = v.copy().normalize();
        }

        for (int i = 0; i < 4; i++) {
            Vector2d angularVelocity = ROTATION_DIRS[i].copy().multiply(angularSpeed);
            Vector2d transVelocity = v.copy().multiply(Math.min(1 - angularSpeed, speed));
            transVelocity.add(angularVelocity);
            double wheelSpeed = transVelocity.dot(ROLLER_DIRS[i]);
            double rpm = maxAchievableRpm * wheelSpeed;
            motors[i].setPower(rpm / motors[i].getMotorType().getMaxRPM());
        }

    }

    /**
     * Stop the motors.
     */
    public void stop() {
        for (DcMotor motor : motors) {
            motor.setPower(0);
        }
    }

    public DcMotor.RunMode getMode() {
        return runMode;
    }

    public void setMode(DcMotor.RunMode mode) {
        runMode = mode;
        for (int i = 0; i < motors.length; i++) {
            motors[i].setMode(mode);
        }
    }

}