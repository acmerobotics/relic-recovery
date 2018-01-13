package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
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

        drive = new MecanumDrive(hardwareMap);
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

        waitForStart();

        jewelTracker.enable();
        vuMarkTracker.enable();

        jewelSlapper.deployArmAndSlapper();

        sleep(500);

        RelicRecoveryVuMark vuMark;
        while (opModeIsActive()) {
            vuMark = vuMarkTracker.getVuMark();
            if (vuMark != RelicRecoveryVuMark.UNKNOWN) {
                vuMarkTracker.disable();
                break;
            }
        }

        AllianceColor leftJewelColor = jewelTracker.getLeftBlue() > jewelTracker.getRightBlue() ?
                AllianceColor.BLUE : AllianceColor.RED;

        if (leftJewelColor == allianceColor) {
            jewelSlapper.setSlapperPosition(JewelSlapper.Position.RIGHT);
        } else {
            jewelSlapper.setSlapperPosition(JewelSlapper.Position.LEFT);
        }

        sleep(500);

        jewelSlapper.stowArmAndSlapper();

        sleep(500);

        followPathSync(new PathBuilder(new Pose2d(48, -48, Math.PI))
            .lineTo(new Vector2d(12, -48))
            .turn(-Math.PI / 2)
            .build());
    }

    private void followPathSync(Path path) {
        drive.followPath(path);
        while (opModeIsActive() && !drive.isFollowingPath()) {
            drive.update();
        }
    }
}
