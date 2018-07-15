package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.cameraoverlay.CameraStreamServer;
import com.acmerobotics.relicrecovery.configuration.MatchType;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.acmerobotics.relicrecovery.vision.FixedJewelTracker;
import com.acmerobotics.library.vision.VuforiaCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaVuMarkTracker;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

public abstract class AutoOpMode extends LinearOpMode {
    public static final long POLL_INTERVAL = 5; // ms
    public static final double LATERAL_BIAS = 2; // in

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

        camera = new VuforiaCamera();
        jewelTracker = new FixedJewelTracker();
        vuMarkTracker = new VuforiaVuMarkTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(vuMarkTracker);
        camera.addTracker(streamServer.getTracker());
        camera.initialize();

        String autoTransition = robot.config.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

        setup();

        displayInitTelemetry();

        waitForStart();

        telemetry.clearAll();
        telemetry.update();

        streamServer.stop();

        if (isStopRequested()) {
            return;
        }

        robot.intake.releaseBar();

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

    protected void lowerArmAndSlapper() {
        if (robot.jewelSlapper.getArmPosition() == JewelSlapper.ArmPosition.DOWN) {
            return;
        }

        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.sleep(0.25);
        robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.CENTER);
        robot.sleep(0.5);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.DOWN);
        robot.sleep(0.5);
    }

    protected void raiseArmAndSlapper() {
        if (robot.jewelSlapper.getArmPosition() == JewelSlapper.ArmPosition.UP) {
            return;
        }

        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.sleep(0.25);
        robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.STOW);
        robot.sleep(0.5);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        robot.sleep(0.25);
    }
}
