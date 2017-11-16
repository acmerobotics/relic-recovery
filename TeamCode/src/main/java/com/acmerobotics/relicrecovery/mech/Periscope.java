package com.acmerobotics.relicrecovery.mech;

import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 11/16/17.
 */

public class Periscope implements Loop {
    private CRServo cameraLift;
    private Servo cameraRotate;
    private DigitalChannel cameraTouch;

    private boolean shouldExtend;

    private Telemetry telemetry;

    public Periscope(HardwareMap map, Telemetry telemetry) {
        cameraLift = map.crservo.get("cameraLift");
        cameraRotate = map.servo.get("cameraRotate");
        cameraTouch = map.digitalChannel.get("cameraTouch");
        cameraTouch.setMode(DigitalChannel.Mode.INPUT);

        this.telemetry = telemetry;

        setPosition(0.5);
    }

    public void setPosition(double position) {
        cameraRotate.setPosition(position);
    }

    public double getPosition() {
        return cameraRotate.getPosition();
    }

    public void extend() {
        shouldExtend = true;
    }

    public boolean isExtended() {
        return cameraTouch.getState();
    }

    public void registerLoops(Looper looper) {
        looper.addLoop(this);
    }

    @Override
    public void onLoop(long timestamp, long dt) {
        if (shouldExtend && !isExtended()) {
            cameraLift.setPower(0.5);
        }

        telemetry.addData("cameraPosition", cameraRotate.getPosition());
        telemetry.addData("cameraShouldExtend", shouldExtend);
        telemetry.addData("cameraIsExtended", isExtended());
    }

}
