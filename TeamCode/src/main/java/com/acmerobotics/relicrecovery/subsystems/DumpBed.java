package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.motion.PIDFController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

@Config
public class DumpBed extends Subsystem {
    public static final double LIFT_HEIGHT = 0; // TODO: determine
    public static final double PULLEY_RADIUS = 0; // TODO: determine

    public static MotionConstraints LIFT_CONSTRAINTS = new MotionConstraints(0, 0, 0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static PIDFCoefficients LIFT_PIDF = new PIDFCoefficients(0, 0, 0, 0, 0);

    public static double LIFT_CALIBRATION_POWER = -0.2;
    public static double LIFT_PROFILE_MIN_POWER = 0.2;

    public static double ROTATE_HEIGHT = 0;

    public static double LEFT_ROTATE_DOWN_POS = 0.39;
    public static double LEFT_ROTATE_UP_POS = 0.86;

    public static double RELEASE_ENGAGE_POS = 0.61;
    public static double RELEASE_DISENGAGE_POS = 0;

    public enum Mode {
        STATIC,
        CALIBRATE,
        MOTION_PROFILE
    }

    private Mode mode = Mode.STATIC;

    private DcMotor liftMotor;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel liftMagneticTouch;

    private MotionProfile profile;
    private PIDFController profileController;

    private boolean liftDumping, releaseEngaged, calibrated, snapToEndpoint, movingDown;
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

        profileController = new PIDFController(LIFT_PIDF);

        liftMotor = map.dcMotor.get("liftMotor");

        dumpRotateLeft = map.servo.get("bedRotateLeft");
        dumpRotateRight = map.servo.get("bedRotateRight");
        dumpRelease = map.servo.get("bedRelease");

        liftMagneticTouch = map.digitalChannel.get("liftMagneticTouch");
        liftMagneticTouch.setMode(DigitalChannel.Mode.INPUT);

        engageRelease();
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

    private void setLiftPower(double power) {
        liftMotor.setPower(power);
    }

    public double getLiftHeight() {
        double position = getEncoderPosition();
        return 2 * Math.PI * PULLEY_RADIUS * position;
    }

    public void setLiftHeight(double height) {
        setEncoderPosition((int) (height / (2 * Math.PI * PULLEY_RADIUS)));
    }

    public void moveLiftToHeight(double height) {
        if (!calibrated) return;
        double startingHeight = getLiftHeight();
        MotionState start = new MotionState(startingHeight, 0, 0, 0, TimestampedData.getCurrentTime());
        MotionGoal goal = new MotionGoal(height, 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, LIFT_CONSTRAINTS);
        mode = Mode.MOTION_PROFILE;
        snapToEndpoint = (height == 0 || height == LIFT_HEIGHT);
        movingDown = height < startingHeight;
        profileController.reset();
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
            if (getLiftHeight() > ROTATE_HEIGHT) {
                setDumpRotation(0.5);
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
        telemetryMap.put("dumpLiftSnap", snapToEndpoint);
        telemetryMap.put("dumpRotation", dumpRotation);

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
            case MOTION_PROFILE: {
                boolean magneticTouchPressed = !liftMagneticTouch.getState();
                telemetryMap.put("dumpTouchPressed", magneticTouchPressed);
                double liftHeight = getLiftHeight();
                telemetryMap.put("dumpLiftHeight", liftHeight);
                if (magneticTouchPressed && Math.abs(liftHeight - profile.start().x) > 1) {
                    setLiftPower(0);
                    mode = Mode.STATIC;
                    if (movingDown) {
                        setLiftHeight(0);
                    } else {
                        setLiftHeight(LIFT_HEIGHT);
                    }
                }
                double timestamp = TimestampedData.getCurrentTime();
                if (timestamp > profile.end().t) {
                    if (snapToEndpoint) {
                        setLiftPower((movingDown ? -1 : 1) * LIFT_PROFILE_MIN_POWER);
                    } else {
                        setLiftPower(0);
                        mode = Mode.STATIC;
                    }
                } else {
                    MotionState motionState = profile.get(timestamp);
                    profileController.setSetpoint(motionState);
                    if (!liftDumping) {
                        if (dumpRotation > 0.25 && liftHeight < ROTATE_HEIGHT) {
                            setDumpRotation(0);
                        } else if (dumpRotation < 0.25 && liftHeight > ROTATE_HEIGHT) {
                            setDumpRotation(0.5);
                        }
                    }
                    double heightError = liftHeight - motionState.x;
                    telemetryMap.put("dumpLiftHeightError", heightError);
                    double liftUpdate = profileController.update(heightError);
                    telemetryMap.put("dumpLiftHeightUpdate", liftUpdate);
                    double liftPower;
                    if (snapToEndpoint) {
                        liftPower = Math.max(liftUpdate, Math.signum(liftUpdate) * LIFT_PROFILE_MIN_POWER);
                    } else {
                        liftPower = liftUpdate;
                    }
                    telemetryMap.put("dumpLiftPower", liftPower);
                    setLiftPower(liftPower);
                }
                break;
            }
        }

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }
}
