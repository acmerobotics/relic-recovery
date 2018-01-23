package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {
    private DcMotor leftIntake, rightIntake;
    private double intakePower;

    private Telemetry telemetry;

    public Intake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftIntake = map.dcMotor.get("leftIntake");
        rightIntake = map.dcMotor.get("rightIntake");
    }

    public void setIntakePower(double intakePower) {
        this.intakePower = intakePower;
        leftIntake.setPower(intakePower);
        rightIntake.setPower(intakePower);
    }

    @Override
    public void update() {
        telemetry.addData("intakePower", intakePower);
    }
}
