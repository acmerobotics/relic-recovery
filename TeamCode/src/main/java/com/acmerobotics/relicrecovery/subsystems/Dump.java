package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/9/18.
 */

@Config
public class Dump {
    public static PIDCoefficients LIFT_PID = new PIDCoefficients();

    /**
     * These are hysteresis thresholds for controlling when the bed rotates from its initial
     * position to slightly elevated to avoid snagging the top of the bot.
     */
    public static final double HYSTERESIS_LOWER_HEIGHT = 3; // in
    public static final double HYSTERESIS_UPPER_HEIGHT = 4; // in

    public static final double CALIBRATION_SPEED = 0.2;

    public static final double LIFT_PULLEY_RADIUS = 1; // in

    public static final double LIFT_TOP_HEIGHT = 5; // in

    public static final double LEFT_ROTATE_DOWN_POS = 0.5;
    public static final double LEFT_ROTATE_UP_POS = 1;

    public static final double RELEASE_ENGAGE_POS = 0.5;
    public static final double RELEASE_DISENGAGE_POS = 0;

    public static final double ACCEPTABLE_HEIGHT_ERROR = 0.1;

    public enum Mode {
        NORMAL,
        PID,
        CALIBRATE
    }

    private Mode mode = Mode.NORMAL, postCalibrationMode;

    private DcMotor dumpLiftLeft, dumpLiftRight;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel dumpLiftBottomTouch;

    private PIDController liftController;

    private double encoderOffset, dumpRotation;

    private boolean liftDown, liftDumping, releaseEngaged, calibrated;

    public Dump(HardwareMap map) {
        dumpLiftLeft = map.dcMotor.get("dumpLiftLeft");
        dumpLiftRight = map.dcMotor.get("dumpLiftRight");

        dumpRotateLeft = map.servo.get("dumpRotateLeft");
        dumpRotateRight = map.servo.get("dumpRotateRight");
        dumpRelease = map.servo.get("dumpRelease");

        dumpLiftBottomTouch = map.digitalChannel.get("dumpLiftBottomTouch");

        liftController = new PIDController(LIFT_PID);

        engageRelease();
        setDumpRotation(0);
    }

    public void liftDown() {
        if (!liftDown || mode == Mode.PID) {
            liftController.setSetpoint(0);
            liftDown = true;
            if (!calibrated) {
                calibrate(Mode.PID);
            } else {
                liftController.reset();
                mode = Mode.PID;
            }
        }
    }

    public void liftUp() {
        if (liftDown || mode == Mode.PID) {
            liftController.setSetpoint(LIFT_TOP_HEIGHT);
            liftDown = false;
            if (!calibrated) {
                calibrate(Mode.PID);
            } else {
                liftController.reset();
                mode = Mode.PID;
            }
        }
    }

    private void setDumpRotation(double rot) {
        if (Math.abs(rot - dumpRotation) < 0.01) return;
        dumpRotation = rot;
        double servoPosition = LEFT_ROTATE_DOWN_POS + rot * (LEFT_ROTATE_UP_POS - LEFT_ROTATE_DOWN_POS);
        dumpRotateLeft.setPosition(servoPosition);
        dumpRotateRight.setPosition(1 - servoPosition);
    }

    private void engageRelease() {
        if (!releaseEngaged) {
            dumpRelease.setPosition(RELEASE_ENGAGE_POS);
            releaseEngaged = true;
        }
    }

    private void disengageRelease() {
        if (releaseEngaged) {
            dumpRelease.setPosition(RELEASE_DISENGAGE_POS);
            releaseEngaged = false;
        }
    }

    public void dump() {
        if (!liftDumping) {
            liftDumping = true;
            setDumpRotation(1);
            disengageRelease();
        }
    }

    public void retract() {
        if (liftDumping) {
            liftDumping = false;
            if (liftDown) {
                setDumpRotation(0);
            } else {
                setDumpRotation(0.4);
            }
            engageRelease();
        }
    }

    private double getPosition() {
        return dumpLiftLeft.getCurrentPosition() - encoderOffset;
    }

    private double getHeight() {
        double position = getPosition();
        double pulleyCircumference = 2 * Math.PI * LIFT_PULLEY_RADIUS;
        return pulleyCircumference * position / dumpLiftLeft.getMotorType().getTicksPerRev();
    }

    public void calibrate(Mode postCalibrationMode) {
        if (!calibrated && postCalibrationMode != Mode.CALIBRATE) {
            dumpLiftLeft.setPower(-CALIBRATION_SPEED);
            dumpLiftRight.setPower(-CALIBRATION_SPEED);
            this.postCalibrationMode = postCalibrationMode;
            mode = Mode.CALIBRATE;
        }
    }

    public void update() {
        switch (mode) {
            case NORMAL:
                // do nothing
                break;
            case PID:
                double height = getHeight();
                if (height > HYSTERESIS_UPPER_HEIGHT) {
                    setDumpRotation(0.4);
                } else if (height < HYSTERESIS_LOWER_HEIGHT) {
                    setDumpRotation(0);
                }
                double error = liftController.getError(height);
                if (Math.abs(error) < ACCEPTABLE_HEIGHT_ERROR) {
                    dumpLiftLeft.setPower(0);
                    dumpLiftRight.setPower(0);
                    mode = Mode.NORMAL;
                } else {
                    double update = liftController.update(error);
                    dumpLiftLeft.setPower(update);
                    dumpLiftRight.setPower(update);
                }
                break;
            case CALIBRATE:
                if (!dumpLiftBottomTouch.getState()) {
                    dumpLiftLeft.setPower(0);
                    dumpLiftRight.setPower(0);
                    encoderOffset = dumpLiftLeft.getCurrentPosition();
                    calibrated = true;
                    mode = postCalibrationMode;
                    if (postCalibrationMode == Mode.PID) {
                        liftController.reset();
                    }
                }
                break;
        }
    }
}
