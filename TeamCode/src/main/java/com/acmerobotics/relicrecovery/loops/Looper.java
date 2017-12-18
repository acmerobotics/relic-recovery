package com.acmerobotics.relicrecovery.loops;

import android.util.Log;

import com.acmerobotics.relicrecovery.drive.TimestampedData;
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
    public static double DEFAULT_LOOP_TIME = 0.1;

    private List<Loop> loops;
    private double loopTime;
    private boolean running;

    private AppUtil appUtil = AppUtil.getInstance();
    private OpModeManagerImpl opModeManager;

    public Looper() {
        this(DEFAULT_LOOP_TIME);
    }

    public Looper(double loopTime) {
        this.loopTime = loopTime;
        loops = new ArrayList<>();
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(appUtil.getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    @Override
    public void run() {
        this.running = true;
        double loopStartTime = TimestampedData.getCurrentTime();
        while (running) {
            Log.i("Looper", "start loop");

            for (Loop loop : loops) {
                loop.onLoop(loopStartTime, loopTime);
            }

            double loopEndTime = loopStartTime + loopTime;
            while (System.currentTimeMillis() > loopEndTime) {
                loopEndTime += loopTime;
                Log.i("Looper", "skipped loop!!");
            }

            try {
                double waitTime = loopEndTime - TimestampedData.getCurrentTime();
                Thread.sleep((int) Math.round(waitTime * 1000));
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
