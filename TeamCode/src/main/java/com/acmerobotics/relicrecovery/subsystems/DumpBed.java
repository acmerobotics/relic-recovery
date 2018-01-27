package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class DumpBed extends Subsystem {
    public static final double LIFT_HEIGHT = 10.75;
    public static final double ENDPOINT_ALLOWANCE_HEIGHT = 0.25;
    public static final double PULLEY_RADIUS = 0.717;

    public static PIDCoefficients LIFT_PID = new PIDCoefficients(-2, 0, 0);

    public static double LIFT_UP_POWER = 1;
    public static double LIFT_DOWN_POWER = -0.4;
    public static double LIFT_CALIBRATION_POWER = -0.2;

    public static double BED_INTERMEDIATE_ROTATION = 0.5;

    public static double LEFT_ROTATE_DOWN_POS = 0.5;
    public static double LEFT_ROTATE_UP_POS = 0.05;

    public static double RELEASE_ENGAGE_POS = 0.61;
    public static double RELEASE_DISENGAGE_POS = 0.11;

    public enum Mode {
        MANUAL,
        PID,
        MOVE_DOWN,
        MOVE_UP,
        CALIBRATE
    }

    private Mode mode = Mode.MANUAL;

    private DcMotor liftMotor;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel liftMagneticTouch;

    private boolean liftDumping, releaseEngaged, liftUp, skipFirstRead, calibrated, missedSensor, movingDownToSensor;
    private double dumpRotation, manualLiftPower;
    private int encoderOffset;

    private PIDController pidController;

    public static final String[] CONDITIONAL_TELEMETRY_KEYS = {
            "dumpMagneticState",
            "dumpLiftHeight",
            "dumpLiftHeightError",
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

        pidController = new PIDController(LIFT_PID);

        liftMotor = map.dcMotor.get("dumpLift");
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);

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

    public int getEncoderPosition() {
        return liftMotor.getCurrentPosition() + encoderOffset;
    }

    public void setEncoderPosition(int position) {
        encoderOffset = position - liftMotor.getCurrentPosition();
    }

    private int inchesToTicks(double inches) {
        double ticksPerRev = liftMotor.getMotorType().getTicksPerRev();
        double circumference = 2 * Math.PI * PULLEY_RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    private double ticksToInches(int ticks) {
        double ticksPerRev = liftMotor.getMotorType().getTicksPerRev();
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * PULLEY_RADIUS * revs;
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
        manualLiftPower = power;
        mode = Mode.MANUAL;
    }

    public double getLiftHeight() {
        return ticksToInches(getEncoderPosition());
    }

    public void setLiftHeight(double height) {
        setEncoderPosition(inchesToTicks(height));
    }

    public void moveUp() {
        if (!calibrated) return;
        if (!liftUp) {
            mode = Mode.MOVE_UP;
            missedSensor = false;
            liftUp = true;
            skipFirstRead = !liftMagneticTouch.getState();
        }
    }

    public void moveDown() {
        if (!calibrated) return;
        if (liftUp) {
            mode = Mode.MOVE_DOWN;
            missedSensor = false;
            liftUp = false;
            skipFirstRead = !liftMagneticTouch.getState();

            if (isDumping()) {
                retract();
            }
        }
    }

    public boolean isLiftUp() {
        return liftUp;
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
                setDumpRotation(BED_INTERMEDIATE_ROTATION);
            } else {
                setDumpRotation(0);
            }
            engageRelease();
        }
    }

    public void update() {
        telemetryMap.put("dumpMode", mode);
        telemetryMap.put("dumpLiftDumping", liftDumping);
        telemetryMap.put("dumpReleaseEngaged", releaseEngaged);
        telemetryMap.put("dumpRotation", dumpRotation);
        telemetryMap.put("dumpLiftUp", liftUp);
        telemetryMap.put("dumpSkipFirstRead", skipFirstRead);

        double liftPower = 0;

        switch (mode) {
            case MANUAL:
                liftPower = this.manualLiftPower;
                break;
            case PID: {
                double liftHeight = getLiftHeight();
                telemetryMap.put("dumpLiftHeight", liftHeight);
                double error = pidController.getError(liftHeight);
                telemetryMap.put("dumpLiftHeightError", error);
                liftPower = pidController.update(error);

                break;
            }
            case MOVE_UP: {
                boolean magneticState = !liftMagneticTouch.getState();
                telemetryMap.put("dumpMagneticState", magneticState);
                double liftHeight = getLiftHeight();
                telemetryMap.put("dumpLiftHeight", liftHeight);

                if (!liftDumping) {
                    setDumpRotation(BED_INTERMEDIATE_ROTATION);
                }

                if (!missedSensor && liftHeight > LIFT_HEIGHT + ENDPOINT_ALLOWANCE_HEIGHT) {
                    missedSensor = true;
                    movingDownToSensor = true;
                }

                if (magneticState && !skipFirstRead) {
                    mode = Mode.PID;
                    setLiftHeight(LIFT_HEIGHT);
                    pidController.reset();
                    pidController.setSetpoint(LIFT_HEIGHT);
                } else if (missedSensor && movingDownToSensor) {
                    if (liftHeight < LIFT_HEIGHT - ENDPOINT_ALLOWANCE_HEIGHT) {
                        movingDownToSensor = false;
                        liftPower = LIFT_UP_POWER;
                    } else {
                        movingDownToSensor = true;
                        liftPower = LIFT_DOWN_POWER;
                    }
                } else if (missedSensor && !movingDownToSensor) {
                    if (liftHeight > LIFT_HEIGHT + ENDPOINT_ALLOWANCE_HEIGHT) {
                        movingDownToSensor = true;
                        liftPower = LIFT_DOWN_POWER;
                    } else {
                        movingDownToSensor = false;
                        liftPower = LIFT_UP_POWER;
                    }
                }  else {
                    liftPower = LIFT_UP_POWER;
                }

                if (!magneticState && skipFirstRead) {
                    skipFirstRead = false;
                }

                break;
            }
            case MOVE_DOWN: {
                boolean magneticTouchPressed = !liftMagneticTouch.getState();
                telemetryMap.put("dumpMagneticState", magneticTouchPressed);
                double liftHeight = getLiftHeight();
                telemetryMap.put("dumpLiftHeight", liftHeight);

                if (!missedSensor && liftHeight < -ENDPOINT_ALLOWANCE_HEIGHT) {
                    missedSensor = true;
                    movingDownToSensor = false;
                }

                if (magneticTouchPressed && !skipFirstRead) {
                    mode = Mode.MANUAL;
                    setLiftHeight(0);
                    if (!liftDumping) {
                        setDumpRotation(0);
                    }
                } else if (missedSensor && movingDownToSensor) {
                    if (liftHeight < -ENDPOINT_ALLOWANCE_HEIGHT) {
                        movingDownToSensor = false;
                        liftPower = LIFT_UP_POWER;
                    } else {
                        movingDownToSensor = true;
                        liftPower = LIFT_DOWN_POWER;
                    }
                } else if (missedSensor && !movingDownToSensor) {
                    if (liftHeight > ENDPOINT_ALLOWANCE_HEIGHT) {
                        movingDownToSensor = true;
                        liftPower = LIFT_DOWN_POWER;
                    } else {
                        movingDownToSensor = false;
                        liftPower = LIFT_UP_POWER;
                    }
                } else {
                    liftPower = LIFT_DOWN_POWER;
                }

                if (!magneticTouchPressed && skipFirstRead) {
                    skipFirstRead = false;
                }

                break;
            }
            case CALIBRATE: {
                boolean magneticTouchPressed = !liftMagneticTouch.getState();
                telemetryMap.put("dumpMagneticState", magneticTouchPressed);

                if (magneticTouchPressed) {
                    mode = Mode.MANUAL;
                    liftUp = false;
                    setLiftHeight(0);
                    calibrated = true;
                } else {
                    liftPower = LIFT_CALIBRATION_POWER;
                }

                break;
            }
        }

        telemetryMap.put("dumpLiftPower", liftPower);
        liftMotor.setPower(liftPower);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }
}
