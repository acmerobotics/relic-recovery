package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbServoController;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class MRServoControllerTester extends Tester<ModernRoboticsUsbServoController> {

    private int port = 1;
    private double servoPosition = 0.5;

    public MRServoControllerTester(String name, ModernRoboticsUsbServoController device) {
        super(name, device);
    }

    @Override
    public String getType() {
        return "MR Servo Controller";
    }

    @Override
    public String getId() {
        return device.getSerialNumber().toString();
    }

    @Override
    public void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry) {
        if (stickyGamepad.a) {
            port = cycleForward(port, 1, 6);
        }
        if (stickyGamepad.b) {
            port = cycleBackward(port, 1, 6);
        }

        if (stickyGamepad.x) {
            servoPosition += 0.01;
        }
        if (stickyGamepad.y) {
            servoPosition -= 0.01;
        }

        servoPosition += gamepad.left_trigger / 300;
        servoPosition -= gamepad.right_trigger / 300;

        servoPosition = Range.clip(servoPosition, 0, 1);

        if (stickyGamepad.left_bumper && stickyGamepad.right_bumper) {
            servoPosition = 0.5;
        } else if (stickyGamepad.left_bumper) {
            servoPosition = 1;
        } else if (stickyGamepad.right_bumper) {
            servoPosition = 0;
        }

        device.setServoPosition(port, servoPosition);

        telemetry.addData("servo (A/B)", port);
        telemetry.addData("position (LT/RT,X/Y)", servoPosition);
    }
}
