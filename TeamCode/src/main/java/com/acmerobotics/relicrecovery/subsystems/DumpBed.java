package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.TelemetryEx;
import com.acmerobotics.relicrecovery.hardware.CachingDcMotor;
import com.acmerobotics.relicrecovery.hardware.CachingServo;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class DumpBed extends Subsystem {
    public static double LIFT_HEIGHT = 10.75;
    public static double LIFT_ENDPOINT_ALLOWANCE_HEIGHT = 0.25;
    public static double LIFT_PULLEY_RADIUS = 0.717;

    public static PIDCoefficients LIFT_PID = new PIDCoefficients(-2, 0, 0);

    public static double LIFT_UP_POWER = 1;
    public static double LIFT_DOWN_POWER = -0.4;
    public static double LIFT_CALIBRATION_POWER = -0.2;
    public static double LIFT_MISSED_SENSOR_MULTIPLIER = 0.25;

    public static double BED_HALFWAY_ROTATION = 0.35;

    public static double BED_LEFT_DOWN_POSITION = 0.14; // .09
    public static double BED_RIGHT_DOWN_POSITION = 0.69;
    public static double BED_LEFT_UP_POSITION = 0.76; // .6
    public static double BED_RIGHT_UP_POSITION = 0.02;

    public static double BED_RELEASE_ENGAGE_POSITION = 0.42;
    public static double BED_RELEASE_DISENGAGE_POSITION = 0.85;

    public enum LiftMode {
        MANUAL,
        HOLD_POSITION,
        MOVE_TO_POSITION,
        MISSED_SENSOR,
        CALIBRATE
    }

    private LiftMode liftMode = LiftMode.MANUAL;

    private DcMotor liftMotor;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel liftHallEffectSensor;

    private boolean bedDumping, liftUp, skipFirstRead, calibrated, movingDownToSensor;

    private double dumpRotation, liftPower;
    private int encoderOffset;

    private PIDController pidController;

    private TelemetryEx telemetry;
    private TelemetryData telemetryData;

    public class TelemetryData {
        public LiftMode dumpLiftMode;
        public boolean dumpLiftDumping;
        public double dumpRotation;
        public boolean dumpLiftUp;
        public boolean dumpSkipFirstRead;

        public double dumpLiftPower;

        public boolean dumpHallEffectState;

        public double dumpLiftHeight;
        public double dumpLiftError;
    }

    public DumpBed(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new TelemetryEx(telemetry);
        this.telemetryData = new TelemetryData();

        pidController = new PIDController(LIFT_PID);

        liftMotor = new CachingDcMotor(map.dcMotor.get("dumpLift"));
        liftMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        liftMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        dumpRotateLeft = new CachingServo(map.servo.get("dumpRotateLeft"));
        dumpRotateRight = new CachingServo(map.servo.get("dumpRotateRight"));
        dumpRelease = new CachingServo(map.servo.get("dumpRelease"));

        liftHallEffectSensor = map.digitalChannel.get("dumpLiftMagneticTouch");
        liftHallEffectSensor.setMode(DigitalChannel.Mode.INPUT);

        retract();
        calibrate();
    }
    
    private boolean isHallEffectSensorTriggered() { 
        return !liftHallEffectSensor.getState();
    }

    public LiftMode getLiftMode() {
        return liftMode;
    }

    public int getEncoderPosition() {
        return liftMotor.getCurrentPosition() + encoderOffset;
    }

    public void setEncoderPosition(int position) {
        encoderOffset = position - liftMotor.getCurrentPosition();
    }

    private int inchesToTicks(double inches) {
        double ticksPerRev = liftMotor.getMotorType().getTicksPerRev();
        double circumference = 2 * Math.PI * LIFT_PULLEY_RADIUS;
        return (int) Math.round(inches * ticksPerRev / circumference);
    }

    private double ticksToInches(int ticks) {
        double ticksPerRev = liftMotor.getMotorType().getTicksPerRev();
        double revs = ticks / ticksPerRev;
        return 2 * Math.PI * LIFT_PULLEY_RADIUS * revs;
    }

    public boolean isCalibrated() {
        return calibrated;
    }

    public void calibrate() {
        liftMode = LiftMode.CALIBRATE;
    }

    public void setLiftPower(double power) {
        liftPower = power;
        liftMode = LiftMode.MANUAL;
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
            liftMode = LiftMode.MOVE_TO_POSITION;
            liftUp = true;
            skipFirstRead = isHallEffectSensorTriggered();
        }
    }

    public void moveDown() {
        if (!calibrated) return;
        if (liftUp) {
            liftMode = LiftMode.MOVE_TO_POSITION;
            liftUp = false;
            skipFirstRead = isHallEffectSensorTriggered();
        }
    }

    public boolean isLiftUp() {
        return liftUp;
    }

    private void setBedRotation(double rot) {
        double leftPosition = BED_LEFT_DOWN_POSITION + (BED_LEFT_UP_POSITION - BED_LEFT_DOWN_POSITION) * rot;
        double rightPosition = BED_RIGHT_DOWN_POSITION + (BED_RIGHT_UP_POSITION - BED_RIGHT_DOWN_POSITION) * rot;
        dumpRotateLeft.setPosition(leftPosition);
        dumpRotateRight.setPosition(rightPosition);
        this.dumpRotation = rot;
    }

    public boolean isDumping() {
        return bedDumping;
    }

    public void dump() {
        bedDumping = true;
    }

    public void retract() {
        bedDumping = false;
    }

    public void update() {
        telemetryData.dumpLiftMode = liftMode;
        telemetryData.dumpLiftDumping = bedDumping;
        telemetryData.dumpRotation = dumpRotation;
        telemetryData.dumpLiftUp = liftUp;
        telemetryData.dumpSkipFirstRead = skipFirstRead;

        double liftPower = 0;

        switch (liftMode) {
            case MANUAL:
                liftPower = this.liftPower;
                break;
            case HOLD_POSITION: {
                double liftHeight = getLiftHeight();
                double error = pidController.getError(liftHeight);
                liftPower = pidController.update(error);

                telemetryData.dumpLiftHeight = liftHeight;
                telemetryData.dumpLiftError = error;

                break;
            }
            case MOVE_TO_POSITION: {
                boolean hallEffectState = isHallEffectSensorTriggered();
//                double liftHeight = getLiftHeight();

                telemetryData.dumpHallEffectState = hallEffectState;
//                telemetryData.dumpLiftHeight = liftHeight;

//                if (liftHeight <= -LIFT_ENDPOINT_ALLOWANCE_HEIGHT || liftHeight >= LIFT_HEIGHT + LIFT_ENDPOINT_ALLOWANCE_HEIGHT) {
//                    liftMode = LiftMode.MISSED_SENSOR;
//                }

                if (!hallEffectState && skipFirstRead) {
                    skipFirstRead = false;
                }

                if (hallEffectState && !skipFirstRead) {
                    liftPower = 0;

                    double snappedHeight = liftUp ? LIFT_HEIGHT : 0;
                    setLiftHeight(snappedHeight);

                    if (liftUp) {
                        liftMode = LiftMode.HOLD_POSITION;

                        pidController.reset();
                        pidController.setSetpoint(snappedHeight);
                    } else {
                        liftMode = LiftMode.MANUAL;
                    }
                } else if (liftUp) {
                    liftPower = LIFT_UP_POWER;
                } else {
                    liftPower = LIFT_DOWN_POWER;
                }

                break;
            }
            case MISSED_SENSOR: {
                boolean hallEffectState = isHallEffectSensorTriggered();
                double liftHeight = getLiftHeight();

                if (hallEffectState) {
                    liftPower = 0;

                    double snappedHeight = liftUp ? LIFT_HEIGHT : 0;
                    setLiftHeight(snappedHeight);

                    if (liftUp) {
                        liftMode = LiftMode.HOLD_POSITION;

                        pidController.reset();
                        pidController.setSetpoint(snappedHeight);
                    } else {
                        liftMode = LiftMode.MANUAL;
                    }
                } else {
                    if (liftHeight <= -LIFT_ENDPOINT_ALLOWANCE_HEIGHT) {
                        liftMode = LiftMode.MISSED_SENSOR;
                        movingDownToSensor = false;
                    } else if (liftHeight >= LIFT_HEIGHT + LIFT_ENDPOINT_ALLOWANCE_HEIGHT) {
                        liftMode = LiftMode.MISSED_SENSOR;
                        movingDownToSensor = true;
                    }

                    if (movingDownToSensor) {
                        liftPower = LIFT_MISSED_SENSOR_MULTIPLIER * LIFT_DOWN_POWER;
                    } else {
                        liftPower = LIFT_MISSED_SENSOR_MULTIPLIER * LIFT_UP_POWER;
                    }
                }

                break;
            }
            case CALIBRATE: {
                boolean hallEffectState = isHallEffectSensorTriggered();

                telemetryData.dumpHallEffectState = hallEffectState;

                if (hallEffectState) {
                    liftMode = LiftMode.MANUAL;
                    liftUp = false;
                    setLiftHeight(0);
                    calibrated = true;
                } else {
                    liftPower = LIFT_CALIBRATION_POWER;
                }

                break;
            }
        }

        telemetryData.dumpLiftPower = liftPower;

        liftMotor.setPower(liftPower);

        if (bedDumping) {
            setBedRotation(1);
            dumpRelease.setPosition(BED_RELEASE_DISENGAGE_POSITION);
        } else if (liftUp) {
            setBedRotation(BED_HALFWAY_ROTATION);
            dumpRelease.setPosition(BED_RELEASE_ENGAGE_POSITION);
        } else {
            setBedRotation(0);
            dumpRelease.setPosition(BED_RELEASE_ENGAGE_POSITION);
        }

        telemetry.addDataObject(telemetryData);
    }
}
