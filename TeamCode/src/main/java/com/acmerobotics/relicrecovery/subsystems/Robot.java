package com.acmerobotics.relicrecovery.subsystems;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.TelemetryPacket;
import com.acmerobotics.library.util.CSVWriter;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.io.File;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

public class Robot implements OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    public interface Listener {
        void onPostUpdate();
    }

    public RobotDashboard dashboard;
    public final OpModeConfiguration config;

    // subsystems
    // note: these *should* be final but it messes with some fancy initialization stuff
    public MecanumDrive drive;
    public Intake intake;
    public DumpBed dumpBed;
    public JewelSlapper jewelSlapper;
    public RelicRecoverer relicRecoverer;

    private List<Subsystem> subsystems;
    private List<Subsystem> subsystemsWithProblems;
    private List<CountDownLatch> cycleLatches;
    private Map<Subsystem, CSVWriter> subsystemLogs;
    private CSVWriter robotLog;
    private OpModeManagerImpl opModeManager;
    private ExecutorService subsystemUpdateExecutor, telemetryUpdateExecutor;
    private BlockingQueue<TelemetryPacket> telemetryPacketQueue;

    private List<Listener> listeners;

    private boolean started;

    private Runnable subsystemUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            TelemetryPacket telemetryPacket = new TelemetryPacket();
            try {
                double startTimestamp = TimestampedData.getCurrentTime();
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        Map<String, Object> telemetry = subsystem.update(telemetryPacket.fieldOverlay());
                        CSVWriter subsystemLog = subsystemLogs.get(subsystem);
                        subsystemLog.putAll(telemetry);
                        subsystemLog.write();
                        telemetryPacket.putAll(telemetry);
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            if (!subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.add(subsystem);
                            }
                        }
                    }
                }
                for (Listener listener : listeners) {
                    listener.onPostUpdate();
                }
                double postSubsystemUpdateTimestamp = TimestampedData.getCurrentTime();
                robotLog.put("subsystemUpdateTime", postSubsystemUpdateTimestamp - startTimestamp);
                robotLog.write();
                while (telemetryPacketQueue.remainingCapacity() == 0) {
                    Thread.sleep(1);
                }
                telemetryPacketQueue.add(telemetryPacket);
                synchronized (cycleLatches) {
                    int i = 0;
                    while (i < cycleLatches.size()) {
                        CountDownLatch latch = cycleLatches.get(i);
                        latch.countDown();
                        if (latch.getCount() == 0) {
                            cycleLatches.remove(i);
                        } else {
                            i++;
                        }
                    }
                }
            } catch (Throwable t) {
                Log.wtf(TAG, t);
            }
        }
    };

    private Runnable telemetryUpdateRunnable = () -> {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                TelemetryPacket packet = telemetryPacketQueue.take();
                dashboard.sendTelemetryPacket(packet);

                for (CSVWriter log : subsystemLogs.values()) {
                    log.flush();
                }
                robotLog.flush();
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }

        }
    };

    public Robot(OpMode opMode) {
        dashboard = RobotDashboard.getInstance();
        config = new OpModeConfiguration(opMode.hardwareMap.appContext);

        listeners = new ArrayList<>();

        File logRoot = LoggingUtil.getLogDir(opMode, config);

        robotLog = new CSVWriter(new File(logRoot, "Robot.csv"));

        subsystems = new ArrayList<>();
        subsystemLogs = new HashMap<>();

        try {
            drive = new MecanumDrive(opMode.hardwareMap);
            subsystemLogs.put(drive, new CSVWriter(new File(logRoot, "Drive.csv")));
            subsystems.add(drive);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping MecanumDrive");
        }

        try {
            intake = new Intake(opMode.hardwareMap);
            subsystemLogs.put(intake, new CSVWriter(new File(logRoot, "Intake.csv")));
            subsystems.add(intake);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Intake");
        }

        try {
            dumpBed = new DumpBed(opMode.hardwareMap);
            subsystemLogs.put(dumpBed, new CSVWriter(new File(logRoot, "DumpBed.csv")));
            subsystems.add(dumpBed);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping DumpBed");
        }

        try {
            jewelSlapper = new JewelSlapper(opMode.hardwareMap);
            subsystemLogs.put(jewelSlapper, new CSVWriter(new File(logRoot, "JewelSlapper.csv")));
            subsystems.add(jewelSlapper);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping JewelSlapper");
        }

        try {
            relicRecoverer = new RelicRecoverer(opMode.hardwareMap);
            subsystemLogs.put(relicRecoverer, new CSVWriter(new File(logRoot, "RelicRecoverer.csv")));
            subsystems.add(relicRecoverer);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping RelicRecoverer");
        }

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        subsystemUpdateExecutor = ThreadPool.newSingleThreadExecutor("subsystem update");
        telemetryUpdateExecutor = ThreadPool.newSingleThreadExecutor("telemetry update");

        telemetryPacketQueue = new ArrayBlockingQueue<>(10);

        subsystemsWithProblems = new ArrayList<>();
        RobotLog.registerGlobalWarningSource(this);

        cycleLatches = new ArrayList<>();
    }

    public void addListener(Listener listener) {
        listeners.add(listener);
    }

    public void start() {
        if (!started) {
            subsystemUpdateExecutor.submit(subsystemUpdateRunnable);
            telemetryUpdateExecutor.submit(telemetryUpdateRunnable);
            started = true;
        }
    }

    private void stop() {
        if (subsystemUpdateExecutor != null) {
            subsystemUpdateExecutor.shutdownNow();
            subsystemUpdateExecutor = null;
        }

        if (telemetryUpdateExecutor != null) {
            telemetryUpdateExecutor.shutdownNow();
            telemetryUpdateExecutor = null;
        }

        RobotLog.unregisterGlobalWarningSource(this);
    }

    public void waitForNextCycle() {
        CountDownLatch latch = new CountDownLatch(1);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }

    public void waitOneFullCycle() {
        CountDownLatch latch = new CountDownLatch(2);
        synchronized (cycleLatches) {
            cycleLatches.add(latch);
        }
        try {
            latch.await();
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
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

    @Override
    public String getGlobalWarning() {
        List<String> warnings = new ArrayList<>();
        synchronized (subsystemsWithProblems) {
            for (Subsystem subsystem : subsystemsWithProblems) {
                warnings.add("Problem with " + subsystem.getClass().getSimpleName());
            }
        }
        return RobotLog.combineGlobalWarnings(warnings);
    }

    @Override
    public void suppressGlobalWarning(boolean suppress) {

    }

    @Override
    public boolean setGlobalWarning(String warning) {
        return false;
    }

    @Override
    public void clearGlobalWarning() {
        synchronized (subsystemsWithProblems) {
            subsystemsWithProblems.clear();
        }
    }

    public void sleep(double seconds) {
        try {
            Thread.sleep(Math.round(1000 * seconds));
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
