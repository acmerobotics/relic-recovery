package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * @author Ryan
 */

public abstract class TimedOpMode extends OpMode implements Loop {
    protected Looper looper;
    private boolean looperStarted;

    public TimedOpMode(int loopMs) {
        looper = new Looper(loopMs);
        msStuckDetectInit = 10000; // give some extra time for init
    }

    @Override
    public final void loop() {

    }

    @Override
    public final void internalPostInitLoop() {
        if (!looperStarted) {
            looper.addLoop((timestamp, dt) -> telemetry.update());
            looper.start();
            looperStarted = true;
        }
    }

    @Override
    public final void internalPostLoop() {

    }
}
