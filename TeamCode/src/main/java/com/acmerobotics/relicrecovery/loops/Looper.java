package com.acmerobotics.relicrecovery.loops;

import android.util.Log;

import com.acmerobotics.relicrecovery.drive.TimestampedData;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class Looper implements Runnable, OpModeManagerNotifier.Notifications {
    public static double DEFAULT_LOOP_TIME = 0.05;

    private List<Loop> loops;
    private double loopTime;
    private boolean running;

    private AppUtil appUtil = AppUtil.getInstance();
    private OpModeManagerImpl opModeManager;
    private ExecutorService executorService;

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
        executorService = ThreadPool.newSingleThreadExecutor("looper - " + loopTime + "s");
    }

    public void start() {
        executorService.execute(this);
    }

    public void stop() {
        if (executorService != null) {
            executorService.shutdownNow();
            executorService = null;
        }
    }

    @Override
    public void run() {
        double loopStartTime = TimestampedData.getCurrentTime();
        while (!Thread.currentThread().isInterrupted()) {
            Log.i("Looper", "start loop");

            for (Loop loop : loops) {
                loop.onLoop(loopStartTime, loopTime);
            }

            double loopEndTime = loopStartTime + loopTime;
            if (TimestampedData.getCurrentTime() > loopEndTime) {
                loopEndTime = TimestampedData.getCurrentTime();
                Log.i("Looper", "cut loop short!!");
            } else {
                try {
                    double waitTime = loopEndTime - TimestampedData.getCurrentTime();
                    Thread.sleep((int) Math.round(waitTime * 1000));
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }

            loopStartTime = loopEndTime;
        }
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
        stop();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}
