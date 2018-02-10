package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.cameraoverlay.CameraStreamServer;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.MatchType;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaVuMarkTracker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public abstract class AutoOpMode extends LinearOpMode {
    public static final double PICTOGRAPH_READ_TIMEOUT = 5;

    protected Robot robot;

    protected VuforiaCamera camera;
    protected FixedJewelTracker jewelTracker;
    protected VuforiaVuMarkTracker vuMarkTracker;

    private CameraStreamServer streamServer;

    protected abstract void setup();
    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.drive.enablePositionEstimation();
        robot.start();

        streamServer = new CameraStreamServer();

//        camera = new VuforiaCamera();
//        jewelTracker = new FixedJewelTracker();
//        vuMarkTracker = new VuforiaVuMarkTracker();
//        camera.addTracker(jewelTracker);
//        camera.addTracker(vuMarkTracker);
//        camera.addTracker(streamServer.getTracker());
//        camera.initialize();

        String autoTransition = robot.config.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

        setup();

        displayInitTelemetry();

        waitForStart();

        streamServer.stop();

        if (isStopRequested()) {
            return;
        }

        run();
    }

    private void displayInitTelemetry() {
        telemetry.addData("matchType", robot.config.getMatchType());
        if (robot.config.getMatchType() != MatchType.PRACTICE) {
            telemetry.addData("matchNumber", robot.config.getMatchNumber());
        }
        telemetry.addData("delay", robot.config.getDelay());
        telemetry.addData("allianceColor", robot.config.getAllianceColor());
        telemetry.addData("balancingStone", robot.config.getBalancingStone());
        telemetry.addData("autoTransition", robot.config.getAutoTransition());
        telemetry.update();
    }

    protected RelicRecoveryVuMark scoreJewelAndReadPictograph() {
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        robot.jewelSlapper.lowerArmAndSlapper();

        sleep(1500);

        double startTime = TimestampedData.getCurrentTime();
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.UNKNOWN;
        while (opModeIsActive()) {
            double timeElapsed = TimestampedData.getCurrentTime() - startTime;
            vuMark = vuMarkTracker.getVuMark();
            if (timeElapsed >= PICTOGRAPH_READ_TIMEOUT || vuMark != RelicRecoveryVuMark.UNKNOWN) {
                break;
            }
        }

        jewelTracker.disable();

        if (jewelPosition != JewelPosition.UNKNOWN) {
            if (robot.config.getAllianceColor() == jewelPosition.rightColor()) {
                // remove left
                robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
            } else {
                // remove right
                robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
            }
        }

        sleep(1500);

        robot.jewelSlapper.stowArmAndSlapper();

        sleep(1500);

        return vuMark;
    }

    protected void followPathSync(Path path) {
        robot.drive.followPath(path);
        waitForPathFollower();
    }

    protected void waitForPathFollower() {
        while (opModeIsActive() && robot.drive.isFollowingPath()) {
            sleep(5);
        }
    }

    protected void alignWithColumnSync() {
//        robot.drive.enableHeadingCorrection();
        robot.drive.alignWithColumn();
        while (opModeIsActive() && robot.drive.getMode() == MecanumDrive.Mode.COLUMN_ALIGN) {
            sleep(5);
        }
//        robot.drive.disableHeadingCorrection();
    }
}
