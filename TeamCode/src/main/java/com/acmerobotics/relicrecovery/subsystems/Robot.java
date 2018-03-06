package com.acmerobotics.relicrecovery.subsystems;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.library.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.GlobalWarningSource;
import com.qualcomm.robotcore.util.RobotLog;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;
import java.util.concurrent.ExecutorService;

public class Robot implements Runnable, OpModeManagerNotifier.Notifications, GlobalWarningSource {
    public static final String TAG = "Robot";

    public interface Listener {
        void onPostUpdate();
    }

    public final RobotDashboard dashboard;
    public final OpModeConfiguration config;

    // subsystems
    // note: these *should* be final but it messes with some fancy initialization stuff
    public MecanumDrive drive;
    public Intake intake;
    public DumpBed dumpBed;
    public JewelSlapper jewelSlapper;
    public RelicRecoverer relicRecoverer;

    private List<Subsystem> subsystems;
    private final List<Subsystem> subsystemsWithProblems;
    private final List<CountDownLatch> cycleLatches;
    private List<Telemetry> allTelemetry;
    private CSVLoggingTelemetry robotTelemetry;
    private OpModeManagerImpl opModeManager;
    private ExecutorService updateExecutor;

    private List<Listener> listeners;

    private boolean started;

    public Robot(OpMode opMode) {
        dashboard = RobotDashboard.getInstance();
        config = new OpModeConfiguration(opMode.hardwareMap.appContext);

        listeners = new ArrayList<>();

        File logRoot = LoggingUtil.getLogDir(opMode, config);

        robotTelemetry = new CSVLoggingTelemetry(new File(logRoot, "Robot.csv"));

        allTelemetry = new ArrayList<>();
        subsystems = new ArrayList<>();

        try {
            CSVLoggingTelemetry driveLogger = new CSVLoggingTelemetry(new File(logRoot, "Drive.csv"));
            drive = new MecanumDrive(opMode.hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(), driveLogger));
            subsystems.add(drive);
            allTelemetry.add(driveLogger);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping MecanumDrive");
        }

        try {
            CSVLoggingTelemetry intakeLogger = new CSVLoggingTelemetry(new File(logRoot, "Intake.csv"));
            intake = new Intake(opMode.hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(), intakeLogger));
            subsystems.add(intake);
            allTelemetry.add(intakeLogger);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping Intake");
        }

        try {
            CSVLoggingTelemetry dumpBedLogger = new CSVLoggingTelemetry(new File(logRoot, "DumpBed.csv"));
            dumpBed = new DumpBed(opMode.hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(), dumpBedLogger));
            subsystems.add(dumpBed);
            allTelemetry.add(dumpBedLogger);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping DumpBed");
        }

        try {
            CSVLoggingTelemetry jewelSlapperLogger = new CSVLoggingTelemetry(new File(logRoot, "JewelSlapper.csv"));
            jewelSlapper = new JewelSlapper(opMode.hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(), jewelSlapperLogger));
            subsystems.add(jewelSlapper);
            allTelemetry.add(jewelSlapperLogger);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping JewelSlapper");
        }

        try {
            CSVLoggingTelemetry relicRecovererLogger = new CSVLoggingTelemetry(new File(logRoot, "RelicRecoverer.csv"));
            relicRecoverer = new RelicRecoverer(opMode.hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(), relicRecovererLogger));
            subsystems.add(relicRecoverer);
            allTelemetry.add(relicRecovererLogger);
        } catch (IllegalArgumentException e) {
            Log.w(TAG, "skipping RelicRecoverer");
        }

        allTelemetry.add(dashboard.getTelemetry());

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        updateExecutor = ThreadPool.newSingleThreadExecutor("robot updater");

        subsystemsWithProblems = new ArrayList<>();
        RobotLog.registerGlobalWarningSource(this);

        cycleLatches = new ArrayList<>();
    }

    public void addListener(Listener listener) {
        listeners.add(listener);
    }

    public void start() {
        if (!started) {
            updateExecutor.submit(this);
            started = true;
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                double startTimestamp = TimestampedData.getCurrentTime();
                for (Subsystem subsystem : subsystems) {
                    if (subsystem == null) continue;
                    try {
                        subsystem.update();
                        synchronized (subsystemsWithProblems) {
                            if (subsystemsWithProblems.contains(subsystem)) {
                                subsystemsWithProblems.remove(subsystem);
                            }
                        }
                    } catch (Throwable t) {
                        Log.w(TAG, "Subsystem update failed for " + subsystem.getClass().getSimpleName() + ": " + t.getMessage());
                        Log.w(TAG, t);
                        synchronized (subsystemsWithProblems) {
                            subsystemsWithProblems.add(subsystem);
                        }
                    }
                }
                for (Listener listener : listeners) {
                    listener.onPostUpdate();
                }
                double postSubsystemUpdateTimestamp = TimestampedData.getCurrentTime();
                for (Telemetry telemetry : allTelemetry) {
                    telemetry.update();
                }
                dashboard.drawOverlay();
                double postTelemetryUpdateTimestamp = TimestampedData.getCurrentTime();
                robotTelemetry.addData("subsystemUpdateTime", postSubsystemUpdateTimestamp - startTimestamp);
                robotTelemetry.addData("telemetryUpdateTime", postTelemetryUpdateTimestamp - postSubsystemUpdateTimestamp);
                robotTelemetry.update();
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
    }

    private void stop() {
        if (updateExecutor != null) {
            updateExecutor.shutdownNow();
            updateExecutor = null;
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
