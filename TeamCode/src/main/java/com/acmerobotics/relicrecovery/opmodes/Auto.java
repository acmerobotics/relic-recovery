package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaVuMarkTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Auto", group = "auto")
public class Auto extends LinearOpMode {
    private Robot robot;

    private VuforiaCamera camera;
    private FixedJewelTracker jewelTracker;
    private VuforiaVuMarkTracker vuMarkTracker;

    @Override
    public void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.dashboard.getTelemetry().setMsTransmissionInterval(50);
        robot.start();

        BalancingStone balancingStone = robot.config.getBalancingStone();
        Path path = AutoPaths.makeNormalPathToCryptobox(balancingStone, RelicRecoveryVuMark.CENTER);
        Pose2d initialPose = path.getPose(0);

        robot.drive.setEstimatedPose(initialPose);

//        camera = new VuforiaCamera();
//        jewelTracker = new FixedJewelTracker();
//        vuMarkTracker = new VuforiaVuMarkTracker();
//        camera.addTracker(jewelTracker);
//        camera.addTracker(vuMarkTracker);
//        camera.addTracker(new FpsTracker());
//        camera.initialize();

        String autoTransition = robot.config.getAutoTransition();
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

        followPathSync(path);
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
        robot.drive.followPath(path);
        while (opModeIsActive() && robot.drive.isFollowingPath()) {
            sleep(5);
        }
        robot.drive.stop();
    }
}
