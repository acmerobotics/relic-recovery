package com.acmerobotics.relicrecovery.opmodes;

import android.app.Activity;
import android.os.Looper;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.acmerobotics.relicrecovery.path.WaitSegment;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.JewelColor;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;

import java.util.Arrays;

/**
 * Kept around for reference
 * @author Ryan
 */

@Deprecated
@Disabled
@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode implements OpModeManagerImpl.Notifications {
    public static final double JEWEL_TURN_ANGLE = Math.toRadians(30);

    private Looper looper;

    private MecanumDrive drive;

    private VuforiaCamera camera;
    private DynamicJewelTracker jewelTracker;

    private OpModeConfiguration configuration;

    private OpModeManagerImpl opModeManager;

    private CSVLoggingTelemetry loggingTelemetry;

    @Override
    public void runOpMode() throws InterruptedException {
        configuration = new OpModeConfiguration(hardwareMap.appContext);

        RobotDashboard dashboard = RobotDashboard.getInstance();

        loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        Telemetry allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        BalancingStone balancingStone = configuration.getBalancingStone();
        Pose2d initialPose = balancingStone.getPose();

        drive = new MecanumDrive(hardwareMap);
        drive.setEstimatedPose(initialPose);

//        looper = new Looper();
//        drive.registerLoops(looper);
//
//        looper.addLoop((timestamp, dt) -> {
//            allTelemetry.update();
//            dashboard.drawOverlay();
//        });

        camera = new VuforiaCamera();
        jewelTracker = new DynamicJewelTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize();

        VuforiaLocalizer vuforia = camera.getVuforia();
        VuforiaTrackables relicTrackables = vuforia.loadTrackablesFromAsset("RelicVuMark");
        VuforiaTrackable relicVuMark = relicTrackables.get(0);
        relicVuMark.setName("relicVuMark");

        relicTrackables.activate();

        AllianceColor allianceColor = configuration.getAllianceColor();

        String autoTransition = configuration.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

//        looper.start();

        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity((Activity) hardwareMap.appContext);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        waitForStart();

        long startTime = System.currentTimeMillis();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive() && (jewelTracker.getLeftColor() == JewelColor.UNKNOWN
                || vuMark == RelicRecoveryVuMark.UNKNOWN) && (System.currentTimeMillis() - startTime) < 7000) {
            vuMark = RelicRecoveryVuMark.from(relicVuMark);

            sleep(10);
        }

//        periscope.stop();

        if ((System.currentTimeMillis() - startTime) < 7000) {
            boolean turnLeft = jewelTracker.getLeftColor().getAllianceColor() != allianceColor;

            double turnAngle = (turnLeft ? 1 : -1) * JEWEL_TURN_ANGLE;
            Path jewelTurnOne = new Path(Arrays.asList(
                    new PointTurn(initialPose, turnAngle)
            ));
            Path jewelTurnTwo = new Path(Arrays.asList(
                    new PointTurn(new Pose2d(initialPose.pos(),
                            Angle.norm(initialPose.heading() + turnAngle)), -turnAngle)
            ));

            drive.followPath(jewelTurnOne);
            waitForPathFollower();


//            startTime = System.currentTimeMillis();
//            while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 1500) {
//                Thread.yield();
//            }
            sleep(1000);

            drive.followPath(jewelTurnTwo);
            waitForPathFollower();
        } else {
            sleep(1000);
        }

        Path cryptoPath = AutoPaths.makePathToCryptobox(balancingStone, vuMark);
        cryptoPath.addSegment(new WaitSegment(cryptoPath.getPose(cryptoPath.duration()), 1));
        drive.followPath(cryptoPath);
        waitForPathFollower();

        sleep(500);

        Path cryptoRetreat = AutoPaths.makeCryptoboxRetreat(balancingStone, vuMark);
        drive.followPath(cryptoRetreat);
        waitForPathFollower();
    }

    private void waitForPathFollower() {
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
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
        MainTeleOp.initialPose = drive.getEstimatedPose();

        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}
