package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 1/11/18.
 */

@Config
public class RelicRecoverer extends Subsystem {
    public static double WRIST_STOW_POSITION = 0.04;
    public static double WRIST_UP_POSITION = 1;
    public static double WRIST_DOWN_POSITION = 0.4;

    public static double FINGER_CLOSE_POSITION = 0.61;
    public static double FINGER_OPEN_POSITION = 0.35;

    private Telemetry telemetry;

    public enum WristPosition {
        STOW,
        UP,
        DOWN
    }

    private DcMotor relicExtend;
    private Servo relicWrist, relicFinger;

    private WristPosition wristPosition;
    private boolean fingerClosed;

    public RelicRecoverer(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        relicExtend = map.dcMotor.get("relicExtend");

        relicWrist = map.servo.get("relicWrist");
        relicFinger = map.servo.get("relicFinger");

        setWristPosition(WristPosition.STOW);
        closeFinger();
    }

    public void setExtendPower(double power) {
        relicExtend.setPower(power);
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

    @Override
    public void update() {
        telemetry.addData("relicWristPosition", wristPosition);
        telemetry.addData("relicFingerClosed", fingerClosed);

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
    }
}
