package com.acmerobotics.relicrecovery.loops;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class Looper extends Thread implements OpModeManagerNotifier.Notifications {
    private List<Loop> loops;
    private long loopMs;
    private boolean running;

    private AppUtil appUtil = AppUtil.getInstance();
    private OpModeManagerImpl opModeManager;

    public Looper(long loopMs) {
        this.loopMs = loopMs;
        loops = new ArrayList<>();
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(appUtil.getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    @Override
    public void run() {
        this.running = true;
        while (running) {
            long startTime = System.currentTimeMillis();

            for (Loop loop : loops) {
                loop.onLoop(startTime, loopMs);
            }

            while (System.currentTimeMillis() - startTime < loopMs) {
                try {
                    Thread.sleep(0, 250);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    public void terminate() {
        this.running = false;
    }

    public void addLoop(Loop loop) {
        this.loops.add(loop);
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        terminate();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
        try {
            join();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
