package com.acmerobotics.velocityvortex.mech;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by kelly on 12/7/2016.
 */

public class Collector {

    private double velocity;
    private DcMotor motor;

    private boolean running;

    public Collector(HardwareMap hardwareMap) {
        motor = hardwareMap.dcMotor.get("collector");
    }

    public void forward() {
        velocity = 1;
        running = true;
        motor.setPower(1);
    }

    public void reverse() {
        velocity = -1;
        running = true;
        motor.setPower(-1);
    }

    public boolean isRunning() {
        return running;
    }

    public void stop() {
        velocity = 0;
        running = false;
        motor.setPower(0);
    }

    public void toggle() {
        if (running) stop();
        else forward();
    }
}
