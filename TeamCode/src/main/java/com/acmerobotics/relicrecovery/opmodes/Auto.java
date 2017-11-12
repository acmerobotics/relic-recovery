package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.configuration.BalancingStone;
import com.acmerobotics.library.configuration.OpModeConfiguration;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.DataFile;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.JewelColor;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.io.File;
import java.util.Arrays;

/**
 * @author Ryan
 */

@Autonomous
public class Auto extends LinearOpMode {
    private RobotDashboard dashboard;
    private Looper looper;

    private MecanumDrive drive;

    private VisionCamera camera;
    private DynamicJewelTracker jewelTracker;

    private OpModeConfiguration configuration;

    @Override
    public void runOpMode() throws InterruptedException {
        configuration = new OpModeConfiguration(hardwareMap.appContext);

        dashboard = RobotDashboard.getInstance();
        String opModeDir = "Auto-" + System.currentTimeMillis();
        drive = new MecanumDrive(hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(),
                new CSVLoggingTelemetry(new File(DataFile.getStorageDir(),
                        opModeDir + File.pathSeparator + "MecanumDrive.csv").getPath())));

        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        looper = new Looper(20);

        drive.registerLoops(looper);

        camera = new VisionCamera(hardwareMap.appContext);
        jewelTracker = new DynamicJewelTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        waitForStart();

        looper.start();

        sleep(configuration.getDelay() * 1000);

        while (opModeIsActive() && jewelTracker.getLeftColor() == JewelColor.UNKNOWN) {
            sleep(10);
        }

        AllianceColor allianceColor = configuration.getAllianceColor();

        BalancingStone balancingStone = configuration.getBalancingStone();
        Pose2d balancingStonePose = balancingStone.getPose();
        Pose2d balancingStonePoseRotated = new Pose2d(balancingStonePose.pos(),
                jewelTracker.getLeftColor().getAllianceColor() == allianceColor ? 3 * Math.PI / 4 : 5 * Math.PI / 4);

        Path jewelTurn = Path.createFromPoses(Arrays.asList(
                balancingStonePose,
                balancingStonePoseRotated
        ));

        drive.followPath(jewelTurn);
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        // TODO: make this alliance/balancing stone agnostic
        Path pathToCryptobox = Path.createFromPoses(Arrays.asList(
                balancingStonePoseRotated,
                new Pose2d(12, -48),
                new Pose2d(12, -60, Math.PI / 2)
        ));

        drive.followPath(pathToCryptobox);
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        while (opModeIsActive()) {
            sleep(10);
        }

        looper.terminate();
        camera.close();
    }
}
