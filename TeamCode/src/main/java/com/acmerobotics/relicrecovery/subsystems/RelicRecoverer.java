package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Created by ryanbrott on 1/11/18.
 */

@Config
public class RelicRecoverer extends Subsystem {
    public static double WRIST_STOW_POSITION = 0.2;
    public static double WRIST_UP_POSITION = 1;
    public static double WRIST_DOWN_POSITION = 0.5;

    public static double FINGER_CLOSE_POSITION = 0.9;
    public static double FINGER_OPEN_POSITION = 0.6;

    public static double ARM_PULLEY_RADIUS = 1.367; // in
    public static double MAX_EXTENSION_DISTANCE = 38;
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

    public static final String[] CONDITIONAL_TELEMETRY_KEYS = {
            "relicArmPosition",
            "relicArmError"
    };

    private Telemetry telemetry;
    private LinkedHashMap<String, Object> telemetryMap;

    private DcMotor relicArm;
    private Servo relicWrist, relicFinger;

    private int encoderOffset;

    private PIDController armController;
    private double armPower;

    private WristPosition wristPosition;
    private boolean fingerClosed;

    private ArmMode armMode = ArmMode.MANUAL;

    public RelicRecoverer(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetryMap = new LinkedHashMap<>();
        for (String key : CONDITIONAL_TELEMETRY_KEYS) {
            telemetryMap.put(key, 0);
        }

        relicArm = map.dcMotor.get("relicArm");

        relicWrist = map.servo.get("relicWrist");
        relicFinger = map.servo.get("relicFinger");

        armController = new PIDController(ARM_PID);

        setWristPosition(WristPosition.STOW);
        closeFinger();
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
        telemetryMap.put("relicWristPosition", wristPosition);
        telemetryMap.put("relicFingerClosed", fingerClosed);

        switch (armMode) {
            case MANUAL:
                break;
            case PID:
                double armPosition = getArmPosition();
                double armError = armController.getError(armPosition);
                armPower = armController.update(armError);

                telemetryMap.put("relicArmPosition", armPosition);
                telemetryMap.put("relicArmError", armError);

                break;
        }

        relicArm.setPower(armPower);

        telemetryMap.put("relicArmMode", armMode);
        telemetryMap.put("relicArmPower", armPower);

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

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }
}
