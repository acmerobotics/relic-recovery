package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.DifferentialControlLoopCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MRMotorControllerTester extends Tester<ModernRoboticsUsbDcMotorController> {

    private int port = 1;
    private DcMotor.RunMode runMode;
    private DcMotor.ZeroPowerBehavior zeroPowerBehavior;

    public MRMotorControllerTester(String name, ModernRoboticsUsbDcMotorController device) {
        super(name, device);
    }

    @Override
    public String getType() {
        return "MR Motor Controller";
    }

    @Override
    public String getId() {
        return device.getSerialNumber().toString();
    }

    @Override
    public void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry) {
        if (stickyGamepad.a) {
            port = cycleForward(port, 1, 2);
        }

        runMode = device.getMotorMode(port);
        if (stickyGamepad.x) {
            runMode = DcMotor.RunMode.values()[cycleForward(runMode.ordinal(), 0, 2)];
        }
        device.setMotorMode(port, runMode);

        zeroPowerBehavior = device.getMotorZeroPowerBehavior(port);
        if (stickyGamepad.y) {
            do {
                zeroPowerBehavior = DcMotor.ZeroPowerBehavior.values()[cycleForward(zeroPowerBehavior.ordinal(), 0, 2)];
            } while (zeroPowerBehavior == DcMotor.ZeroPowerBehavior.UNKNOWN);
        }
        device.setMotorZeroPowerBehavior(port, zeroPowerBehavior);

        double power = -gamepad.left_stick_y;
        device.setMotorPower(port, power);

        telemetry.addData("port (A)", port);
        telemetry.addData("mode (X)", runMode);
        telemetry.addData("zero_behavior (Y)", zeroPowerBehavior);
        telemetry.addData("power (LJ)", power);
        telemetry.addData("encoder", device.getMotorCurrentPosition(port));
        telemetry.addData("voltage", Math.round(100.0 * device.getVoltage()) / 100.0 + " V");
        DifferentialControlLoopCoefficients pid = device.getDifferentialControlLoopCoefficients(port);
        telemetry.addData("pid", pid.p + "," + pid.i + "," + pid.d);
        telemetry.addData("gear_ratio", device.getGearRatio(port));
    }
}
