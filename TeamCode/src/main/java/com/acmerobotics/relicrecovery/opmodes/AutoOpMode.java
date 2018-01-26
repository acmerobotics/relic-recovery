package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaVuMarkTracker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

public abstract class AutoOpMode extends LinearOpMode {
    public static final double PICTOGRAPH_READ_TIMEOUT = 5;

    protected Robot robot;

    protected VuforiaCamera camera;
    protected DynamicJewelTracker jewelTracker;
    protected VuforiaVuMarkTracker vuMarkTracker;

    protected abstract void setup();
    protected abstract void run();

    @Override
    public final void runOpMode() throws InterruptedException {
        robot = new Robot(this);
        robot.start();

        camera = new VuforiaCamera();
        jewelTracker = new DynamicJewelTracker();
        vuMarkTracker = new VuforiaVuMarkTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(vuMarkTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize();

        String autoTransition = robot.config.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

        setup();

        displayInitTelemetry();

        waitForStart();

        run();
    }

    private void displayInitTelemetry() {
        telemetry.addData("matchType", robot.config.getMatchType());
        telemetry.addData("matchNumber", robot.config.getMatchNumber());
        telemetry.addData("delay", robot.config.getDelay());
        telemetry.addData("allianceColor", robot.config.getAllianceColor());
        telemetry.addData("balancingStone", robot.config.getBalancingStone());
        telemetry.addData("autoTransition", robot.config.getAutoTransition());
        telemetry.update();
    }

    protected RelicRecoveryVuMark scoreJewelAndReadPictograph() {
        jewelTracker.enable();

        robot.jewelSlapper.deployArmAndSlapper();

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

        JewelPosition jewelPosition = jewelTracker.getJewelPosition();

        jewelTracker.disable();

        if (jewelPosition != JewelPosition.UNKNOWN) {
            if (robot.config.getAllianceColor() == jewelPosition.rightColor()) {
                // remove left
                robot.jewelSlapper.setSlapperPosition(JewelSlapper.Position.LEFT);
            } else {
                // remove right
                robot.jewelSlapper.setSlapperPosition(JewelSlapper.Position.RIGHT);
            }
        }

        sleep(1500);

        robot.jewelSlapper.stowArmAndSlapper();

        sleep(1500);

        return vuMark;
    }

    protected void followPathSync(Path path) {
        robot.drive.followPath(path);
        while (opModeIsActive() && robot.drive.isFollowingPath()) {
            sleep(5);
        }
    }
}