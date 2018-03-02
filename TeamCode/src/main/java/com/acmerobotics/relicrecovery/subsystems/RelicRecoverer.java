package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.TelemetryEx;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 1/11/18.
 */

@Config
public class RelicRecoverer extends Subsystem {
    public static double WRIST_STOW_POSITION = 0.36;
    public static double WRIST_UP_POSITION = 0.9;
    public static double WRIST_DOWN_POSITION = 0.36;

    public static double FINGER_CLOSE_POSITION = 0;
    public static double FINGER_OPEN_POSITION = 0.45;

    public static double ARM_PULLEY_RADIUS = 2.734; // in
    public static double MAX_EXTENSION_DISTANCE = 46;
    public static PIDCoefficients ARM_PID = new PIDCoefficients(-1.5, 0, -0.02);

    public enum ArmMode {
        PID,
        MANUAL
    }

    public enum WristPosition {
        STOW,
        UP,
        DOWN
    }

    private DcMotor relicArm;
    private Servo relicWrist, relicFinger;

    private int encoderOffset;

    private PIDController armController;
    private double armPower;

    private WristPosition wristPosition;
    private boolean fingerClosed;

    private ArmMode armMode = ArmMode.MANUAL;

    private TelemetryEx telemetry;
    private TelemetryData telemetryData;

    private class TelemetryData {
        public WristPosition relicWristPosition;
        public boolean relicFingerClosed;
        public ArmMode relicArmMode;
        public double relicArmPower;
        public double relicArmPosition;
        public double relicArmError;
    }

    public RelicRecoverer(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new TelemetryEx(telemetry);
        telemetryData = new TelemetryData();

        relicArm = map.dcMotor.get("relicArm");
        relicArm.setDirection(DcMotorSimple.Direction.REVERSE);

        relicWrist = map.servo.get("relicWrist");
        relicFinger = map.servo.get("relicFinger");

        armController = new PIDController(ARM_PID);

        setWristPosition(WristPosition.STOW);
        openFinger();
        resetEncoder();
    }

    public void setArmPower(double power) {
        armPower = power;
        armMode = ArmMode.MANUAL;
    }

    public void setArmPosition(double distance) {
        armController.setSetpoint(distance);
        armController.reset();
        armMode = ArmMode.PID;
    }

    public void resetEncoder() {
        encoderOffset = relicArm.getCurrentPosition();
    }

    public int getEncoderPosition() {
        return relicArm.getCurrentPosition() - encoderOffset;
    }

    public double getArmPosition() {
        int encoderPosition = getEncoderPosition();
        double revs = encoderPosition / relicArm.getMotorType().getTicksPerRev();
        return 2 * Math.PI * ARM_PULLEY_RADIUS * revs;
    }

    public void setWristPosition(WristPosition position) {
        wristPosition = position;
    }

    public void closeFinger() {
        fingerClosed = true;
    }

    public void openFinger() {
        fingerClosed = false;
    }

    public ArmMode getArmMode() {
        return armMode;
    }

    @Override
    public void update() {
        telemetryData.relicWristPosition = wristPosition;
        telemetryData.relicFingerClosed = fingerClosed;

        switch (armMode) {
            case MANUAL:
                break;
            case PID:
                double armPosition = getArmPosition();
                double armError = armController.getError(armPosition);
                armPower = armController.update(armError);

                telemetryData.relicArmPosition = armPosition;
                telemetryData.relicArmError = armError;

                break;
        }

        relicArm.setPower(armPower);

        telemetryData.relicArmMode = armMode;
        telemetryData.relicArmPower = armPower;

        switch (wristPosition) {
            case STOW:
                relicWrist.setPosition(WRIST_STOW_POSITION);
                break;
            case UP:
                relicWrist.setPosition(WRIST_UP_POSITION);
                break;
            case DOWN:
                relicWrist.setPosition(WRIST_DOWN_POSITION);
                break;
        }

        if (fingerClosed) {
            relicFinger.setPosition(FINGER_CLOSE_POSITION);
        } else {
            relicFinger.setPosition(FINGER_OPEN_POSITION);
        }

        telemetry.addDataObject(telemetryData);
    }
}
