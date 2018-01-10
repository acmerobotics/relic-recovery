package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PwmControl;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.ServoImplEx;

/**
 * Created by ryanbrott on 1/9/18.
 */

// TODO: implement lifting
public class Dump implements Loop {
    private PriorityScheduler scheduler;

    private Servo dumpRelease;
    private ServoImplEx dumpRotateLeft, dumpRotateRight;

    private boolean down, releaseEngaged;

    public Dump(HardwareMap map, PriorityScheduler scheduler) {
        this.scheduler = scheduler;

        dumpRotateLeft = map.get(ServoImplEx.class, "dumpRotateLeft");
        dumpRotateLeft.setPwmRange(new PwmControl.PwmRange(500, 2500, 50000));
        dumpRotateRight = map.get(ServoImplEx.class, "dumpRotateRight");
        dumpRotateRight.setPwmRange(new PwmControl.PwmRange(500, 2500, 50000));
        dumpRelease = map.servo.get("dumpRelease");

        rotateDown();
        engageRelease();
    }

    public void rotateDown() {
        if (!down) {
            scheduler.add(() -> dumpRotateLeft.setPosition(0.3), "dump: left rotate set pos", PriorityScheduler.HIGH_PRIORITY);
            scheduler.add(() -> dumpRotateRight.setPosition(0.7), "dump: right rotate set pos", PriorityScheduler.HIGH_PRIORITY);
            down = true;
        }
    }

    public void rotateUp() {
        if (down) {
            scheduler.add(() -> dumpRotateLeft.setPosition(0.7), "dump: left rotate set pos", PriorityScheduler.HIGH_PRIORITY);
            scheduler.add(() -> dumpRotateRight.setPosition(0.3), "dump: right rotate set pos", PriorityScheduler.HIGH_PRIORITY);
            down = false;
        }
    }

    public boolean isDown() {
        return down;
    }

    public void engageRelease() {
        if (!releaseEngaged) {
            scheduler.add(() -> dumpRelease.setPosition(0.5), "dump: release set pos", PriorityScheduler.HIGH_PRIORITY);
            releaseEngaged = true;
        }
    }

    public void disengageRelease() {
        if (releaseEngaged) {
            scheduler.add(() -> dumpRelease.setPosition(0), "dump: release set pos", PriorityScheduler.HIGH_PRIORITY);
            releaseEngaged = false;
        }
    }

    public boolean isReleaseEngaged() {
        return releaseEngaged;
    }

    public void registerLoops(Looper looper) {
        looper.addLoop(this);
    }

    @Override
    public void onLoop(double timestamp, double dt) {

    }
}
