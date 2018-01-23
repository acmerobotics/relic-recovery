package com.acmerobotics.relicrecovery.subsystems;

import android.app.Activity;
import android.util.Log;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.io.File;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

public class Robot implements Runnable, OpModeManagerNotifier.Notifications {
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
    private MultipleTelemetry allTelemetry;
    private CSVLoggingTelemetry robotTelemetry;
    private OpModeManagerImpl opModeManager;
    private ExecutorService updateExecutor;
    private OpMode opMode;

    private boolean started;

    public Robot(OpMode opMode) {
        this.opMode = opMode;
        opMode.telemetry.setMsTransmissionInterval(50);

        dashboard = RobotDashboard.getInstance();
        config = new OpModeConfiguration(opMode.hardwareMap.appContext);

        File logRoot = LoggingUtil.getLogRoot(opMode);

        robotTelemetry = new CSVLoggingTelemetry(new File(logRoot, "Robot.csv"));

        List<Telemetry> allTelemetryList = new ArrayList<>();
        subsystems = new ArrayList<>();

        try {
            CSVLoggingTelemetry driveLogger = new CSVLoggingTelemetry(new File(logRoot, "Drive.csv"));
            drive = new MecanumDrive(opMode.hardwareMap, new MultipleTelemetry(driveLogger, dashboard.getTelemetry()));
            subsystems.add(drive);
            allTelemetryList.add(driveLogger);
        } catch (IllegalArgumentException e) {
            Log.w("Robot", "skipping MecanumDrive");
        }

        try {
            CSVLoggingTelemetry intakeLogger = new CSVLoggingTelemetry(new File(logRoot, "Intake.csv"));
            intake = new Intake(opMode.hardwareMap, new MultipleTelemetry(intakeLogger, dashboard.getTelemetry()));
            subsystems.add(intake);
            allTelemetryList.add(intakeLogger);
        } catch (IllegalArgumentException e) {
            Log.w("Robot", "skipping Intake");
        }

        try {
            CSVLoggingTelemetry dumpBedLogger = new CSVLoggingTelemetry(new File(logRoot, "DumpBed.csv"));
            dumpBed = new DumpBed(opMode.hardwareMap, new MultipleTelemetry(dumpBedLogger, dashboard.getTelemetry()));
            subsystems.add(dumpBed);
            allTelemetryList.add(dumpBedLogger);
        } catch (IllegalArgumentException e) {
            Log.w("Robot", "skipping DumpBed");
        }

        try {
            CSVLoggingTelemetry jewelSlapperLogger = new CSVLoggingTelemetry(new File(logRoot, "JewelSlapper.csv"));
            jewelSlapper = new JewelSlapper(opMode.hardwareMap, new MultipleTelemetry(jewelSlapperLogger, dashboard.getTelemetry()));
            subsystems.add(jewelSlapper);
            allTelemetryList.add(jewelSlapperLogger);
        } catch (IllegalArgumentException e) {
            Log.w("Robot", "skipping JewelSlapper");
        }

        try {
            CSVLoggingTelemetry relicRecovererLogger = new CSVLoggingTelemetry(new File(logRoot, "RelicRecoverer.csv"));
            relicRecoverer = new RelicRecoverer(opMode.hardwareMap, new MultipleTelemetry(relicRecovererLogger, dashboard.getTelemetry()));
            subsystems.add(relicRecoverer);
            allTelemetryList.add(relicRecovererLogger);
        } catch (IllegalArgumentException e) {
            Log.w("Robot", "skipping RelicRecoverer");
        }

        allTelemetryList.add(dashboard.getTelemetry());
        Telemetry[] telemetryArray = new Telemetry[allTelemetryList.size()];
        allTelemetry = new MultipleTelemetry(allTelemetryList.toArray(telemetryArray));

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
                if (subsystem == null) continue;
                subsystem.update();
            }
            double postSubsystemUpdateTimestamp = TimestampedData.getCurrentTime();
            allTelemetry.update();
            dashboard.drawOverlay();
            double postTelemetryUpdateTimestamp = TimestampedData.getCurrentTime();
            robotTelemetry.addData("subsystemUpdateTime", postSubsystemUpdateTimestamp - startTimestamp);
            robotTelemetry.addData("telemetryUpdateTime", postTelemetryUpdateTimestamp - postSubsystemUpdateTimestamp);
            robotTelemetry.update();
            opMode.telemetry.addData("x", Math.random());
            opMode.telemetry.update();
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
