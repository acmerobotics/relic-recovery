package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class DumpBed extends Subsystem {
    public static PIDCoefficients LIFT_PID = new PIDCoefficients(-0.01, -0.005, 0);

    public static double LIFT_UP_POWER = 0.5;
    public static double LIFT_DOWN_POWER = -0.2;

    public static double LIFT_DUMP_ROTATION = 0.7;

    public static double LEFT_ROTATE_DOWN_POS = 0.54;
    public static double LEFT_ROTATE_UP_POS = 0;

    public static double RELEASE_ENGAGE_POS = 0.61;
    public static double RELEASE_DISENGAGE_POS = 0;

    public enum Mode {
        STATIC,
        PID,
        MOVE_DOWN,
        MOVE_UP
    }

    private Mode mode = Mode.STATIC;

    private DcMotor liftMotor;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel liftMagneticTouch;

    private boolean liftDumping, releaseEngaged, liftUp, skipFirstRead;
    private double dumpRotation;

    private PIDController pidController;

    public static final String[] CONDITIONAL_TELEMETRY_KEYS = {
            "dumpMagneticState",
            "dumpLiftEncoder",
            "dumpLiftEncoderError",
            "dumpLiftEncoderUpdate",
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

        dumpRotateLeft = map.servo.get("dumpRotateLeft");
        dumpRotateRight = map.servo.get("dumpRotateRight");
        dumpRelease = map.servo.get("dumpRelease");

        liftMagneticTouch = map.digitalChannel.get("dumpLiftMagneticTouch");
        liftMagneticTouch.setMode(DigitalChannel.Mode.INPUT);

        engageRelease();
        setDumpRotation(0);
        liftUp = true;
        moveDown();
    }

    public Mode getMode() {
        return mode;
    }

    public int getEncoderPosition() {
        return liftMotor.getCurrentPosition();
    }

    public void moveUp() {
        if (!liftUp) {
            mode = Mode.MOVE_UP;
            liftUp = true;
            skipFirstRead = !liftMagneticTouch.getState();
        }
    }

    public void moveDown() {
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
        telemetryMap.put("dumpLiftDumping", liftDumping);
        telemetryMap.put("dumpReleaseEngaged", releaseEngaged);
        telemetryMap.put("dumpRotation", dumpRotation);
        telemetryMap.put("dumpLiftUp", liftUp);
        telemetryMap.put("dumpSkipFirstRead", skipFirstRead);

        double liftPower = 0;

        switch (mode) {
            case STATIC:
                // do nothing
                break;
            case PID: {
                int encoderPosition = getEncoderPosition();
                telemetryMap.put("dumpLiftEncoderPosition", encoderPosition);
                double error = pidController.getError(encoderPosition);
                telemetryMap.put("dumpLiftEncoderError", error);
                double update = pidController.update(error);
                telemetryMap.put("dumpLiftEncoderUpdate", update);

                liftPower = update;

                break;
            }
            case MOVE_UP: {
                boolean magneticState = !liftMagneticTouch.getState();
                telemetryMap.put("dumpMagneticState", magneticState);

                if (!liftDumping) {
                    setDumpRotation(LIFT_DUMP_ROTATION);
                }

                if (magneticState && !skipFirstRead) {
                    mode = Mode.PID;
                    pidController.reset();
                    pidController.setSetpoint(getEncoderPosition());
                } else {
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

                if (magneticTouchPressed && !skipFirstRead) {
                    mode = Mode.PID;
                    pidController.reset();
                    pidController.setSetpoint(getEncoderPosition());
                    if (!liftDumping) {
                        setDumpRotation(0);
                    }
                } else {
                    liftPower = LIFT_DOWN_POWER;
                }

                if (!magneticTouchPressed && skipFirstRead) {
                    skipFirstRead = false;
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
