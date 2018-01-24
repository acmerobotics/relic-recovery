package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class DumpBed extends Subsystem {
    public static final double LIFT_HEIGHT = 10.75;
    public static final double PULLEY_RADIUS = 0.717;

    public static double LIFT_UP_POWER = 0.5;
    public static double LIFT_DOWN_POWER = -0.2;

    public static double LIFT_DUMP_ROTATION = 0.7;

    public static double LIFT_CALIBRATION_POWER = -0.1;

    public static double LEFT_ROTATE_DOWN_POS = 0.54;
    public static double LEFT_ROTATE_UP_POS = 0;

    public static double RELEASE_ENGAGE_POS = 0.61;
    public static double RELEASE_DISENGAGE_POS = 0;

    public enum Mode {
        STATIC,
        CALIBRATE,
        MOVE_DOWN,
        MOVE_UP
    }

    private Mode mode = Mode.STATIC;

    private DcMotor liftMotor;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel liftMagneticTouch;

    private boolean liftDumping, releaseEngaged, calibrated, liftUp, skipFirstRead;
    private double encoderOffset, dumpRotation;

    public static final String[] CONDITIONAL_TELEMETRY_KEYS = {
            "dumpTouchPressed",
            "dumpLiftHeight",
            "dumpLiftHeightError",
            "dumpLiftHeightUpdate",
            "dumpLiftPower"
    };

    private Telemetry telemetry;
    private LinkedHashMap<String, Object> telemetryMap;

    public DumpBed(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetryMap = new LinkedHashMap<>();
        for (String key : CONDITIONAL_TELEMETRY_KEYS) {
            telemetryMap.put(key, 0);
        }

        liftMotor = map.dcMotor.get("dumpLift");

        dumpRotateLeft = map.servo.get("dumpRotateLeft");
        dumpRotateRight = map.servo.get("dumpRotateRight");
        dumpRelease = map.servo.get("dumpRelease");

        liftMagneticTouch = map.digitalChannel.get("dumpLiftMagneticTouch");
        liftMagneticTouch.setMode(DigitalChannel.Mode.INPUT);

        engageRelease();
        setDumpRotation(0);
        calibrate();
    }

    public Mode getMode() {
        return mode;
    }

    public double getEncoderPosition() {
        return liftMotor.getCurrentPosition() - encoderOffset;
    }

    public void setEncoderPosition(int position) {
        encoderOffset = liftMotor.getCurrentPosition() - position;
    }

    public void resetEncoder() {
        setEncoderPosition(0);
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    public void calibrate() {
        if (!calibrated) {
            mode = Mode.CALIBRATE;
        }
    }

    public void setLiftPower(double power) {
        liftMotor.setPower(power);
    }

    public double getLiftHeight() {
        double position = getEncoderPosition();
        double ticksPerRev = liftMotor.getMotorType().getTicksPerRev();
        return 2 * Math.PI * PULLEY_RADIUS * position / ticksPerRev;
    }

    public void setLiftHeight(double height) {
        double ticksPerRev = liftMotor.getMotorType().getTicksPerRev();
        setEncoderPosition((int) (height * ticksPerRev / (2 * Math.PI * PULLEY_RADIUS)));
    }

    public void moveUp() {
        if (!isCalibrated()) return;
        if (!liftUp) {
            mode = Mode.MOVE_UP;
            liftUp = true;
            skipFirstRead = !liftMagneticTouch.getState();
        }
    }

    public void moveDown() {
        if (!isCalibrated()) return;
        if (liftUp) {
            mode = Mode.MOVE_DOWN;
            liftUp = false;
            skipFirstRead = !liftMagneticTouch.getState();
        }
    }

    private void setDumpRotation(double rot) {
        double servoPosition = LEFT_ROTATE_DOWN_POS + rot * (LEFT_ROTATE_UP_POS - LEFT_ROTATE_DOWN_POS);
        dumpRotateLeft.setPosition(servoPosition);
        dumpRotateRight.setPosition(1 - servoPosition);
        this.dumpRotation = rot;
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

    public boolean isDumping() {
        return liftDumping;
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
            if (liftUp) {
                setDumpRotation(LIFT_DUMP_ROTATION);
            } else {
                setDumpRotation(0);
            }
            engageRelease();
        }
    }

    public void update() {
        telemetryMap.put("dumpMode", mode);
        telemetryMap.put("dumpCalibrated", calibrated);
        telemetryMap.put("dumpLiftDumping", liftDumping);
        telemetryMap.put("dumpReleaseEngaged", releaseEngaged);
        telemetryMap.put("dumpRotation", dumpRotation);
        telemetryMap.put("dumpLiftUp", liftUp);
        telemetryMap.put("dumpSkipFirstRead", skipFirstRead);

        switch (mode) {
            case STATIC:
                // do nothing
                break;
            case CALIBRATE: {
                boolean magneticTouchPressed = !liftMagneticTouch.getState();
                telemetryMap.put("dumpTouchPressed", magneticTouchPressed);
                if (magneticTouchPressed) {
                    setLiftPower(0);
                    resetEncoder();
                    calibrated = true;
                    mode = Mode.STATIC;
                } else {
                    setLiftPower(LIFT_CALIBRATION_POWER);
                }
                break;
            }
            case MOVE_UP: {
                boolean magneticTouchPressed = !liftMagneticTouch.getState();
                telemetryMap.put("dumpTouchPressed", magneticTouchPressed);
                double liftHeight = getLiftHeight();
                telemetryMap.put("dumpLiftHeight", liftHeight);

                if (!liftDumping) {
                    setDumpRotation(LIFT_DUMP_ROTATION);
                }

                if (magneticTouchPressed && !skipFirstRead) {
                    setLiftPower(0);
                    mode = Mode.STATIC;
                } else {
                    setLiftPower(LIFT_UP_POWER);
                }

                if (!magneticTouchPressed && skipFirstRead) {
                    skipFirstRead = false;
                }

                break;
            }
            case MOVE_DOWN: {
                boolean magneticTouchPressed = !liftMagneticTouch.getState();
                telemetryMap.put("dumpTouchPressed", magneticTouchPressed);
                double liftHeight = getLiftHeight();
                telemetryMap.put("dumpLiftHeight", liftHeight);

                if (magneticTouchPressed && !skipFirstRead) {
                    setLiftPower(0);
                    mode = Mode.STATIC;
                    if (!liftDumping) {
                        setDumpRotation(0);
                    }
                } else {
                    setLiftPower(LIFT_DOWN_POWER);
                }

                if (!magneticTouchPressed && skipFirstRead) {
                    skipFirstRead = false;
                }

                break;
            }
        }

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }
}
