package com.acmerobotics.relicrecovery.loops;

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
    public static final int DEFAULT_CAPACITY = 20;

    public interface Task {
        void execute();
    }

    public class TaskWithPriority {
        public final Task task;
        public final int priority;
        public final boolean repeat;
        public final double timestamp;

        public TaskWithPriority(Task task, int priority, boolean repeat) {
            this.task = task;
            this.priority = priority;
            this.repeat = repeat;
            this.timestamp = TimestampedData.getCurrentTime();
        }

        public TaskWithPriority copy() {
            return new TaskWithPriority(task, priority, repeat);
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
        queue = new PriorityBlockingQueue<>(capacity, (lhs, rhs) -> {
            if (lhs.priority == rhs.priority) {
                return Double.compare(lhs.timestamp, rhs.timestamp);
            } else {
                return Integer.compare(lhs.priority, rhs.priority);
            }
        });
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
                TaskWithPriority taskWithPriority = queue.take();
                taskWithPriority.task.execute();
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

    public void add(Task task, int priority) {
        add(new TaskWithPriority(task, priority, false));
    }

    public void addRepeating(Task task, int priority) {
        add(new TaskWithPriority(task, priority, true));
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
