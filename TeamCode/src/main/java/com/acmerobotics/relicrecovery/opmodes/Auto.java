package com.acmerobotics.relicrecovery.opmodes;

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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackable;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaTrackables;

import java.util.Arrays;

/**
 * @author Ryan
 */

@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode {
    public static final double JEWEL_TURN_ANGLE = Math.toRadians(30);

    private Looper looper;

    private MecanumDrive drive;
    private JewelSlapper jewelSlapper;
    private GlyphGripper glyphGripper;
    private Periscope periscope;

    private VisionCamera camera;
    private DynamicJewelTracker jewelTracker;

    private OpModeConfiguration configuration;

    @Override
    public void runOpMode() throws InterruptedException {
        configuration = new OpModeConfiguration(hardwareMap.appContext);

        RobotDashboard dashboard = RobotDashboard.getInstance();

        CSVLoggingTelemetry loggingTelemetry = new CSVLoggingTelemetry(LoggingUtil.getLogFile(this, configuration));
        Telemetry subsystemTelemetry = new MultipleTelemetry(loggingTelemetry, dashboard.getTelemetry());
        Telemetry allTelemetry = new MultipleTelemetry(telemetry, loggingTelemetry, dashboard.getTelemetry());
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        BalancingStone balancingStone = configuration.getBalancingStone();
        Pose2d initialPose = balancingStone.getPose();

        drive = new MecanumDrive(hardwareMap, subsystemTelemetry, initialPose);
        jewelSlapper = new JewelSlapper(hardwareMap);
        glyphGripper = new GlyphGripper(hardwareMap);
        periscope = new Periscope(hardwareMap, subsystemTelemetry);

        looper = new Looper(20);
        drive.registerLoops(looper);
        periscope.registerLoops(looper);

        looper.addLoop((timestamp, dt) -> {
            allTelemetry.update();
            dashboard.drawOverlay();
        });

        glyphGripper.grip();

        camera = new VisionCamera(hardwareMap.appContext);
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

        waitForStart();

        sleep(configuration.getDelay() * 1000);

        jewelSlapper.jewelSlapperDown();
        periscope.raise();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive() && (jewelTracker.getLeftColor() == JewelColor.UNKNOWN
                || vuMark == RelicRecoveryVuMark.UNKNOWN)) {
            vuMark = RelicRecoveryVuMark.from(relicVuMark);

            sleep(10);
        }

        periscope.stop();

        boolean turnLeft = jewelTracker.getLeftColor().getAllianceColor() != allianceColor;
        double turnAngle = (turnLeft ? 1 : -1) * JEWEL_TURN_ANGLE;
        Path jewelTurn = new Path(Arrays.asList(
                new PointTurn(initialPose, turnAngle),
                new PointTurn(new Pose2d(initialPose.pos(),
                        Angle.norm(initialPose.heading() + turnAngle)), -turnAngle)
        ));

        drive.followPath(jewelTurn);
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        jewelSlapper.jewelSlapperUp();

        sleep(500);

        Path cryptoPath = AutoPaths.makePathToCryptobox(balancingStone, vuMark);
        cryptoPath.addSegment(new WaitSegment(cryptoPath.getPose(cryptoPath.duration()), 1));
        drive.followPath(cryptoPath);
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        glyphGripper.release();

        looper.terminate();
        camera.close();
        loggingTelemetry.close();

        MainTeleOp.initialPose = drive.getEstimatedPose();
    }
}
