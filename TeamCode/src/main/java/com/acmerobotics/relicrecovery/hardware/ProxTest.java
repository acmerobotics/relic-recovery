package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

@Autonomous(name="ProxTest")
public class ProxTest extends LinearOpMode{

    private AnalogInput[] inputs;

    @Override
    public void runOpMode() {
        inputs = new AnalogInput[4];

        for (int i = 0; i < inputs.length; i++) {
            inputs[i] = hardwareMap.analogInput.get("a" + i);
        }

        waitForStart();

        while (!isStopRequested()) {
            for (int i = 0; i < inputs.length; i++) {
                telemetry.addData("" + i, inputs[i].getVoltage());
            }
        }
    }
}
