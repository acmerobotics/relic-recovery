package com.acmerobotics.relicrecovery.loops;

import android.util.Log;

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
    public static int DEFAULT_LOOP_MS = 100;

    private List<Loop> loops;
    private long loopMs;
    private boolean running;

    private AppUtil appUtil = AppUtil.getInstance();
    private OpModeManagerImpl opModeManager;

    public Looper() {
        this(DEFAULT_LOOP_MS);
    }

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
        long loopStartTime = System.currentTimeMillis();
        while (running) {
            Log.i("Looper", "start loop");

            for (Loop loop : loops) {
                loop.onLoop(loopStartTime, loopMs);
            }

            long loopEndTime = loopStartTime + loopMs;
            while (System.currentTimeMillis() > loopEndTime) {
                loopEndTime += loopMs;
                Log.i("Looper", "skipped loop!!");
            }

            try {
                Thread.sleep(loopEndTime - System.currentTimeMillis());
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

            loopStartTime = loopEndTime;
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
