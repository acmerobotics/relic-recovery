package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/11/18.
 */

public class RelicRecoverer {
    public enum WristPosition {
        STOW(0.93),
        VERTICAL(0.5),
        HORIZONTAL(0);

        private double position;

        WristPosition(double position) {
            this.position = position;
        }

        public double getServoPosition() {
            return position;
        }
    }

    private DcMotor relicExtend;
    private Servo relicWrist, relicFinger;

    private boolean fingerClosed;

    private WristPosition wristPosition;

    public RelicRecoverer(HardwareMap map) {
        relicExtend = map.dcMotor.get("relicExtend");

        relicWrist = map.servo.get("relicWrist");
        relicFinger = map.servo.get("relicFinger");
    }

    public void setExtendPower(double power) {
        relicExtend.setPower(power);
    }

    public void setWristPosition(WristPosition position) {
        if (wristPosition != position) {
            relicWrist.setPosition(position.getServoPosition());
            wristPosition = position;
        }
    }

    public void closeFinger() {
        if (!fingerClosed) {
            relicFinger.setPosition(0);
            fingerClosed = true;
        }
    }

    public void openFinger() {
        if (fingerClosed) {
            relicFinger.setPosition(0.5);
            fingerClosed = false;
        }
    }
}
