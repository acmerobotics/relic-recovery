package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

/**
 * Created by ryanbrott on 1/7/18.
 */

public abstract class ScheduledLoopOpMode extends OpMode implements Loop {
    protected PriorityScheduler scheduler;
    protected Looper looper;
    private double loopTime;

    public ScheduledLoopOpMode(double loopTime) {
        this.loopTime = loopTime;
    }

    @Override
    public final void init() {
        scheduler = new PriorityScheduler();
        looper = new Looper(scheduler, loopTime);

        looper.addLoop(this);

        setup();

        looper.addLoop((timestamp, dt) -> postLoop());

        scheduler.start();
        looper.start();
    }

    @Override
    public final void init_loop() {
        // intentionally do nothing
    }

    @Override
    public final void loop() {
        // intentionally do nothing
    }

    @Override
    public final void internalPostInitLoop() {
        // intentionally do nothing
    }

    @Override
    public final void internalPostLoop() {
        // intentionally do nothing
    }

    protected void postLoop() {
        telemetry.update();
    }

    protected abstract void setup();
}
