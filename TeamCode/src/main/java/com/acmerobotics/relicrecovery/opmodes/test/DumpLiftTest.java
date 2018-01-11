package com.acmerobotics.relicrecovery.opmodes.test;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

/**
 * Created by ryanbrott on 1/10/18.
 */

@TeleOp(name = "DumpLiftTest")
public class DumpLiftTest extends OpMode {
    private DcMotor dumpLiftLeft, dumpLiftRight;

    @Override
    public void init() {
        dumpLiftLeft = hardwareMap.dcMotor.get("dumpLiftLeft");
        dumpLiftRight = hardwareMap.dcMotor.get("dumpLiftRight");
    }

    @Override
    public void loop() {
        dumpLiftLeft.setPower(-gamepad1.left_stick_y);
        dumpLiftRight.setPower(-gamepad1.left_stick_y);
    }
}
