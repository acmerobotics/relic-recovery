package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.StickyGamepad;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@Disabled
@TeleOp
public class RelicTest extends OpMode {
    private Robot robot;
    private StickyGamepad stickyGamepad1;

    @Override
    public void init() {
        robot = new Robot(this);
        robot.start();

        stickyGamepad1 = new StickyGamepad(gamepad1);
    }

    @Override
    public void loop() {
        stickyGamepad1.update();

        robot.relicRecoverer.setArmPower(-gamepad1.left_stick_y);

        if (stickyGamepad1.dpad_up) {
            robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);
        } else if (stickyGamepad1.dpad_down) {
            robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.DOWN);
        } else if (stickyGamepad1.dpad_left || stickyGamepad1.dpad_right) {
            robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.STOW);
        }

        if (stickyGamepad1.y) {
            robot.relicRecoverer.openFinger();
        } else if (stickyGamepad1.b) {
            robot.relicRecoverer.closeFinger();
        }
    }
}
