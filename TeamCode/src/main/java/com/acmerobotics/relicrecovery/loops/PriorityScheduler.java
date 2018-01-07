package com.acmerobotics.relicrecovery.loops;

import android.util.Log;

import com.acmerobotics.library.util.TimestampedData;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.util.concurrent.ExecutorService;
import java.util.concurrent.PriorityBlockingQueue;

/**
 * Created by ryanbrott on 1/6/18.
 */

public class PriorityScheduler implements Runnable, OpModeManagerNotifier.Notifications {
    public static final int DEFAULT_CAPACITY = 30;

    public static final int HIGH_PRIORITY = 1;
    public static final int MEDIUM_PRIORITY = 3;
    public static final int LOW_PRIORITY = 5;

    /** amount of time required to make a low priority task become high priority */
    public static final double DECAY_TIME = 0.2;
    public static final double K = Math.log((double) HIGH_PRIORITY / LOW_PRIORITY) / DECAY_TIME;

    public static final boolean DEBUG = true;

    public interface Task {
        void execute();
    }

    public class TaskWithPriority {
        public final Task task;
        public final String name;
        public final int priority;
        public final boolean repeat;
        public final double timestamp;

        public TaskWithPriority(Task task, String name, int priority, boolean repeat) {
            this.task = task;
            this.name = name;
            this.priority = priority;
            this.repeat = repeat;
            this.timestamp = TimestampedData.getCurrentTime();
        }

        public TaskWithPriority copy() {
            return new TaskWithPriority(task, name, priority, repeat);
        }

        public double getAdjustedPriority() {
            double timeSinceQueue = TimestampedData.getCurrentTime() - timestamp;
            return priority * Math.exp(K * timeSinceQueue);
        }
    }

    private ExecutorService executorService;
    private OpModeManagerImpl opModeManager;
    private PriorityBlockingQueue<TaskWithPriority> queue;

    public PriorityScheduler() {
        this(DEFAULT_CAPACITY);
    }

    public PriorityScheduler(int capacity) {
        executorService = ThreadPool.newSingleThreadExecutor("priority scheduler");
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
        queue = new PriorityBlockingQueue<>(capacity,
                (lhs, rhs) -> Double.compare(lhs.getAdjustedPriority(), rhs.getAdjustedPriority()));
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
        while (!Thread.currentThread().isInterrupted()) {
            try {
                for (TaskWithPriority task : queue) {
                    Log.i("Scheduler", String.format("[%1.2f] %s", task.getAdjustedPriority(), task.name));
                }
                TaskWithPriority taskWithPriority = queue.take();
                double startTime = TimestampedData.getCurrentTime();
                taskWithPriority.task.execute();
                double elapsedTime = TimestampedData.getCurrentTime() - startTime;
                if (DEBUG) {
                    String name = taskWithPriority.name.length() == 0 ? "unknown" : taskWithPriority.name;
                    Log.i("Scheduler", String.format("Executed '%s': took %.2fms, queued for %.2fms",
                            name, elapsedTime * 1000, (startTime - taskWithPriority.timestamp) * 1000));
                }
                if (taskWithPriority.repeat) {
                    add(taskWithPriority.copy());
                }
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
    }

    private void add(TaskWithPriority taskWithPriority) {
        queue.add(taskWithPriority);
    }

    public void add(Task task, String name, int priority) {
        add(new TaskWithPriority(task, name, priority, false));
    }

    public void addRepeating(Task task, String name, int priority) {
        add(new TaskWithPriority(task, name, priority, true));
    }

    public double getHighestPriority() {
        TaskWithPriority taskWithPriority = queue.peek();
        return taskWithPriority == null ? LOW_PRIORITY : taskWithPriority.getAdjustedPriority();
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
            opModeManager = null;
        }
    }
}
