package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.subsystems.DumpBed;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaVuMarkTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

/**
 * @author Ryan
 */

@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode {
    private MecanumDrive drive;
    private JewelSlapper jewelSlapper;
    private DumpBed dumpBed;

    private VuforiaCamera camera;
    private FixedJewelTracker jewelTracker;
    private VuforiaVuMarkTracker vuMarkTracker;

    private OpModeConfiguration configuration;

    @Override
    public void runOpMode() throws InterruptedException {
        configuration = new OpModeConfiguration(hardwareMap.appContext);

        BalancingStone balancingStone = configuration.getBalancingStone();
        Pose2d initialPose = balancingStone.getPose();

        drive = new MecanumDrive(hardwareMap, telemetry);
        drive.enablePositionEstimation();
        drive.setEstimatedPose(initialPose);
        jewelSlapper = new JewelSlapper(hardwareMap);
        dumpBed = new DumpBed(hardwareMap, telemetry);

        camera = new VuforiaCamera();
        jewelTracker = new FixedJewelTracker();
        vuMarkTracker = new VuforiaVuMarkTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(vuMarkTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize();

        AllianceColor allianceColor = configuration.getAllianceColor();

        String autoTransition = configuration.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

        dumpBed.liftDown();
        while (!isStopRequested() && !isStarted()) {
            dumpBed.update();
        }

        waitForStart();

        while (opModeIsActive() && dumpBed.getMode() != DumpBed.Mode.NORMAL) {
            dumpBed.update();
        }

        jewelTracker.enable();
        vuMarkTracker.enable();

        jewelSlapper.deployArmAndSlapper();

        sleep(1500);

        double startTime = TimestampedData.getCurrentTime();

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive() && (TimestampedData.getCurrentTime() - startTime) < 8) {
            vuMark = vuMarkTracker.getVuMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                break;
            }
        }
        vuMarkTracker.disable();

        AllianceColor leftJewelColor = jewelTracker.getLeftBlue() > jewelTracker.getRightBlue() ?
                AllianceColor.BLUE : AllianceColor.RED;

        if (leftJewelColor == allianceColor) {
            jewelSlapper.setSlapperPosition(JewelSlapper.Position.RIGHT);
        } else {
            jewelSlapper.setSlapperPosition(JewelSlapper.Position.LEFT);
        }

        sleep(1500);

        jewelSlapper.stowArmAndSlapper();

//        sleep(1500);
//
//        followPathSync(AutoPaths.makePathToCryptobox(balancingStone, vuMark));
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
        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.update();
        }
        drive.stop();
    }
}
