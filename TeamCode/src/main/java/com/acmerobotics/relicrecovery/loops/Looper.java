package com.acmerobotics.relicrecovery.loops;

import android.support.annotation.Nullable;
import android.util.Log;

import com.acmerobotics.library.util.TimestampedData;
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
    public static final double DEFAULT_LOOP_TIME = 0.05;
    public static final double PRIORITY_THRESHOLD = 2;

    private List<Loop> loops;
    private double loopTime;

    private AppUtil appUtil = AppUtil.getInstance();
    private OpModeManagerImpl opModeManager;
    private ExecutorService executorService;
    @Nullable private PriorityScheduler scheduler;

    public Looper() {
        this(null, DEFAULT_LOOP_TIME);
    }

    public Looper(PriorityScheduler scheduler, double loopTime) {
        this.scheduler = scheduler;
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
        double lastLoopDt = loopTime;
        while (!Thread.currentThread().isInterrupted()) {
            Log.i("Looper", "start loop");

            for (Loop loop : loops) {
                loop.onLoop(loopStartTime, lastLoopDt);
            }

            if (scheduler != null) {
                while (scheduler.getHighestPriority() < PRIORITY_THRESHOLD) {
                    try {
                        Thread.sleep(2);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }

            double loopEndTime = loopStartTime + loopTime;
            double currentTime = TimestampedData.getCurrentTime();
            lastLoopDt = currentTime - loopStartTime;
            Log.i("Looper", "end loop: took " + 1000 * lastLoopDt + "ms");
            if (currentTime > loopEndTime) {
                loopEndTime = currentTime;
                Log.w("Looper", "loop took too long!");
            } else {
                try {
                    double waitTime = loopEndTime - currentTime;
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
