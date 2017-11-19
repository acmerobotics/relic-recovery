package com.acmerobotics.relicrecovery.opmodes;

import android.app.Activity;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.configuration.BalancingStone;
import com.acmerobotics.library.configuration.OpModeConfiguration;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.mech.GlyphGripper;
import com.acmerobotics.relicrecovery.mech.GlyphLift;
import com.acmerobotics.relicrecovery.mech.JewelSlapper;
import com.acmerobotics.relicrecovery.mech.Periscope;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.acmerobotics.relicrecovery.path.WaitSegment;
import com.acmerobotics.relicrecovery.util.LoggingUtil;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.JewelColor;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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
 * @author Ryan
 */

@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode implements OpModeManagerImpl.Notifications {
    public static final double JEWEL_TURN_ANGLE = Math.toRadians(30);

    private Looper looper;

    private MecanumDrive drive;
    private JewelSlapper jewelSlapper;
    private GlyphGripper glyphGripper;
    private Periscope periscope;
    private GlyphLift glyphLift;

    private VisionCamera camera;
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

        drive = new MecanumDrive(hardwareMap, subsystemTelemetry, initialPose);
        jewelSlapper = new JewelSlapper(hardwareMap);
        glyphGripper = new GlyphGripper(hardwareMap);
        periscope = new Periscope(hardwareMap, subsystemTelemetry);
        glyphLift = new GlyphLift(hardwareMap, subsystemTelemetry, GlyphLift.Side.FRONT);

        looper = new Looper(20);
        drive.registerLoops(looper);
        periscope.registerLoops(looper);
        glyphLift.registerLoops(looper);

        looper.addLoop((timestamp, dt) -> {
            allTelemetry.update();
            dashboard.drawOverlay();
        });

        glyphGripper.grip();

        camera = new VisionCamera(hardwareMap.appContext);
        camera.setImageDir(LoggingUtil.getImageDir(this));
        jewelTracker = new DynamicJewelTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

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

        looper.start();

        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity((Activity) hardwareMap.appContext);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }

        waitForStart();

        periscope.raise();

        glyphLift.zeroLift();

        while (opModeIsActive() && glyphLift.isLifterZeroing()) {
            sleep(10);
        }

        sleep(configuration.getDelay() * 1000);

        jewelSlapper.jewelSlapperDown();

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

            jewelSlapper.jewelSlapperUp();

//            startTime = System.currentTimeMillis();
//            while (opModeIsActive() && (System.currentTimeMillis() - startTime) < 1500) {
//                Thread.yield();
//            }
            sleep(1000);

            drive.followPath(jewelTurnTwo);
            waitForPathFollower();
        } else {
            jewelSlapper.jewelSlapperUp();

            sleep(1000);
        }

        Path cryptoPath = AutoPaths.makePathToCryptobox(balancingStone, vuMark);
        cryptoPath.addSegment(new WaitSegment(cryptoPath.getPose(cryptoPath.duration()), 1));
        drive.followPath(cryptoPath);
        waitForPathFollower();

        glyphGripper.release();

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
        looper.terminate();
        camera.close();
        loggingTelemetry.close();

        MainTeleOp.initialPose = drive.getEstimatedPose();

        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}
