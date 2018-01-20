package com.acmerobotics.relicrecovery.subsystems;

import android.app.Activity;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class Robot implements Runnable, OpModeManagerNotifier.Notifications {
    public final RobotDashboard dashboard;
    public final OpModeConfiguration config;

    // subsystems
    public final Intake intake;
    public final DumpBed dumpBed;
    public final JewelSlapper jewelSlapper;
    public final RelicRecoverer relicRecoverer;

    private List<Subsystem> subsystems;
    private MultipleTelemetry allTelemetry;
    private CSVLoggingTelemetry robotTelemetry;
    private OpModeManagerImpl opModeManager;
    private ExecutorService updateExecutor;

    private boolean started;

    public Robot(OpMode opMode) {
        dashboard = RobotDashboard.getInstance();
        config = new OpModeConfiguration(opMode.hardwareMap.appContext);

        File logRoot = LoggingUtil.getLogRoot(opMode);

        robotTelemetry = new CSVLoggingTelemetry(new File(logRoot, "Robot.csv"));

        CSVLoggingTelemetry intakeLogger = new CSVLoggingTelemetry(new File(logRoot, "Intake.csv"));
        intake = new Intake(opMode.hardwareMap, new MultipleTelemetry(intakeLogger, dashboard.getTelemetry()));

        CSVLoggingTelemetry dumpBedLogger = new CSVLoggingTelemetry(new File(logRoot, "DumpBed.csv"));
        dumpBed = new DumpBed(opMode.hardwareMap, new MultipleTelemetry(dumpBedLogger, dashboard.getTelemetry()));

        CSVLoggingTelemetry jewelSlapperLogger = new CSVLoggingTelemetry(new File(logRoot, "JewelSlapper.csv"));
        jewelSlapper = new JewelSlapper(opMode.hardwareMap, new MultipleTelemetry(jewelSlapperLogger, dashboard.getTelemetry()));

        CSVLoggingTelemetry relicRecovererLogger = new CSVLoggingTelemetry(new File(logRoot, "RelicRecoverer.csv"));
        relicRecoverer = new RelicRecoverer(opMode.hardwareMap, new MultipleTelemetry(relicRecovererLogger, dashboard.getTelemetry()));

        subsystems = new ArrayList<>(Arrays.asList(intake, dumpBed, jewelSlapper, relicRecoverer));
        allTelemetry = new MultipleTelemetry(intakeLogger, dumpBedLogger, jewelSlapperLogger, relicRecovererLogger, dashboard.getTelemetry());

        Activity activity = (Activity) opMode.hardwareMap.appContext;
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        updateExecutor = ThreadPool.newSingleThreadExecutor("robot subsystem updater");
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
            double startTimestamp = TimestampedData.getCurrentTime();
            for (Subsystem subsystem : subsystems) {
                subsystem.update();
            }
            double postSubsystemUpdateTimestamp = TimestampedData.getCurrentTime();
            allTelemetry.update();
            double postTelemetryUpdateTimestamp = TimestampedData.getCurrentTime();
            robotTelemetry.addData("subsystemUpdateTime", postSubsystemUpdateTimestamp - startTimestamp);
            robotTelemetry.addData("telemetryUpdateTime", postTelemetryUpdateTimestamp - postSubsystemUpdateTimestamp);
            robotTelemetry.update();
        }
    }

    private void stop() {
        if (updateExecutor != null) {
            updateExecutor.shutdownNow();
            updateExecutor = null;
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
}
