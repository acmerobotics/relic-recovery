package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 1/11/18.
 */

public class RelicRecoverer extends Subsystem {
    private Telemetry telemetry;

    public enum WristPosition {
        STOW(0.04),
        UP(1),
        DOWN(0.5);

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
        relicWrist.setPosition(position.getServoPosition());
    }

    public void closeFinger() {
        relicFinger.setPosition(0.61);
    }

    public void openFinger() {
        relicFinger.setPosition(0.35);
    }

    @Override
    public void update() {

    }
}
