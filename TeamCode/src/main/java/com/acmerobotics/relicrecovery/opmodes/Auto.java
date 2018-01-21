package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaVuMarkTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode {
    private RobotDashboard dashboard;
    private MecanumDrive drive;
//    private JewelSlapper jewelSlapper;
//    private DumpBed dumpBed;

    private VuforiaCamera camera;
    private FixedJewelTracker jewelTracker;
    private VuforiaVuMarkTracker vuMarkTracker;

    private OpModeConfiguration configuration;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = RobotDashboard.getInstance();
        dashboard.getTelemetry().setMsTransmissionInterval(50);
        configuration = new OpModeConfiguration(hardwareMap.appContext);

        BalancingStone balancingStone = configuration.getBalancingStone();
        Vector2d initialPosition = balancingStone.getPosition();

        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry());
        drive.setEstimatedPose(new Pose2d(initialPosition, Math.PI));
//        jewelSlapper = new JewelSlapper(hardwareMap);
//        dumpBed = new DumpBed(hardwareMap, telemetry);

//        camera = new VuforiaCamera();
//        jewelTracker = new FixedJewelTracker();
//        vuMarkTracker = new VuforiaVuMarkTracker();
//        camera.addTracker(jewelTracker);
//        camera.addTracker(vuMarkTracker);
//        camera.addTracker(new FpsTracker());
//        camera.initialize();
//
//        AllianceColor allianceColor = configuration.getAllianceColor();

        String autoTransition = configuration.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

//        dumpBed.liftDown();
//        while (!isStopRequested() && !isStarted()) {
//            dumpBed.update();
//        }

        waitForStart();

//        while (opModeIsActive() && dumpBed.getMode() != DumpBed.Mode.NORMAL) {
//            dumpBed.update();
//        }

//        jewelTracker.enable();
//        vuMarkTracker.enable();

//        jewelSlapper.deployArmAndSlapper();

//        sleep(1500);
//
//        double startTime = TimestampedData.getCurrentTime();
//
//        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
//        while (opModeIsActive() && (TimestampedData.getCurrentTime() - startTime) < 8) {
//            vuMark = vuMarkTracker.getVuMark();
//            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
//                break;
//            }
//        }
//        vuMarkTracker.disable();

//        AllianceColor leftJewelColor = jewelTracker.getLeftBlue() > jewelTracker.getRightBlue() ?
//                AllianceColor.BLUE : AllianceColor.RED;

//        if (leftJewelColor == allianceColor) {
//            jewelSlapper.setSlapperPosition(JewelSlapper.Position.RIGHT);
//        } else {
//            jewelSlapper.setSlapperPosition(JewelSlapper.Position.LEFT);
//        }
//
//        sleep(1500);
//
//        jewelSlapper.stowArmAndSlapper();

        sleep(1500);

        followPathSync(AutoPaths.makeNormalPathToCryptobox(balancingStone, RelicRecoveryVuMark.CENTER));
//
//        dumpBed.liftDown();
//        startTime = TimestampedData.getCurrentTime();
//        while (opModeIsActive() && (TimestampedData.getCurrentTime() - startTime) < 1.5) {
//            dumpBed.update();
//        }
//        dumpBed.dump();
//        sleep(1500);
//        dumpBed.retract();
//        sleep(1500);
    }

    private void followPathSync(Path path) {
        drive.followPath(path);
        int i = 0;
        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.update();
            dashboard.getTelemetry().update();
            if (i % 4 == 0) {
                RobotDashboard.getInstance().drawOverlay();
            }
            i++;
        }
        drive.stop();
    }
}
