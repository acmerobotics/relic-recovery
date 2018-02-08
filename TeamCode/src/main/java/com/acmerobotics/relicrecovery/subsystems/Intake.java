package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Intake extends Subsystem {
    private DcMotor leftIntake, rightIntake;
    private double leftIntakePower, rightIntakePower;

    private Telemetry telemetry;

    public Intake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        leftIntake = map.dcMotor.get("intakeLeft");
        rightIntake = map.dcMotor.get("intakeRight");
    }

    public void setIntakePower(double intakePower) {
        setIntakePower(intakePower, intakePower);
    }

    public void setIntakePower(double leftIntakePower, double rightIntakePower) {
        this.leftIntakePower = leftIntakePower;
        this.rightIntakePower = rightIntakePower;
        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
    }

    @Override
    public void update() {
        telemetry.addData("leftIntakePower", leftIntakePower);
        telemetry.addData("rightIntakePower", rightIntakePower);
    }
}
