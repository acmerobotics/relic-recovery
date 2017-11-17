package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.mech.RelicRecoverer;
import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

/**
 * @author kellyrm
 */

@TeleOp(name="Relic Test")
public class RelicRecovererTest extends OpMode{

    private RelicRecoverer relicRecoverer;
    StickyGamepad gamepad;

    public void init() {
        telemetry = new MultipleTelemetry(RobotDashboard.getInstance().getTelemetry(), telemetry);
        relicRecoverer = new RelicRecoverer(hardwareMap);
        gamepad = new StickyGamepad(gamepad1);
    }

    public void loop() {
        telemetry.addData("extension", relicRecoverer.getExtension());
        telemetry.addData("offset", relicRecoverer.getOffset());
        telemetry.addData("extension error", relicRecoverer.getExtendError());
        telemetry.addData("offset error", relicRecoverer.getOffsetError());
        gamepad.update();
        if (gamepad.dpad_up) relicRecoverer.setPosition(RelicRecoverer.Position.UP);
        if (gamepad.dpad_left) relicRecoverer.setPosition(RelicRecoverer.Position.OPEN);
        if (gamepad.dpad_down) relicRecoverer.setPosition(RelicRecoverer.Position.CLOSED);
        relicRecoverer.setExtension(relicRecoverer.getExtensionTarget() + gamepad1.right_stick_y*5);
        relicRecoverer.setOffset(relicRecoverer.getOffsetTarget() + gamepad1.left_stick_y*5);
        relicRecoverer.onLoop(0, 0);
    }
}
