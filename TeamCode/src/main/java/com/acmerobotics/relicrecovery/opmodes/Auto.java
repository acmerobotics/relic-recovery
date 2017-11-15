package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.configuration.BalancingStone;
import com.acmerobotics.library.configuration.OpModeConfiguration;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.acmerobotics.relicrecovery.vision.DynamicJewelTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.JewelColor;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Arrays;

/**
 * @author Ryan
 */

@Autonomous(name = "Auto", group = "auto")
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
//        String opModeDir = "Auto-" + System.currentTimeMillis();
//        drive = new MecanumDrive(hardwareMap, new MultipleTelemetry(dashboard.getTelemetry(),
//                new CSVLoggingTelemetry(new File(DataFile.getStorageDir(),
//                        opModeDir + File.separator + "MecanumDrive.csv").getPath())));

        drive = new MecanumDrive(hardwareMap, new MultipleTelemetry(dashboard.getTelemetry()));

//        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        looper = new Looper(20);

        drive.registerLoops(looper);

        looper.addLoop(new Loop() {
            @Override
            public void onLoop(long timestamp, long dt) {
                telemetry.update();
            }
        });

        camera = new VisionCamera(hardwareMap.appContext);
        jewelTracker = new DynamicJewelTracker();
        camera.addTracker(jewelTracker);
        camera.addTracker(new FpsTracker());
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        AllianceColor allianceColor = configuration.getAllianceColor();

        BalancingStone balancingStone = configuration.getBalancingStone();
        Pose2d balancingStonePose = balancingStone.getPose().copy();
        drive.setEstimatedPose(balancingStonePose);
        drive.setHeading(balancingStonePose.heading());

        looper.start();

        String autoTransition = configuration.getAutoTransition();
        if (!autoTransition.equals(OpModeConfiguration.NO_AUTO_TRANSITION)) {
            AutoTransitioner.transitionOnStop(this, autoTransition);
        }

        waitForStart();

        sleep(configuration.getDelay() * 1000);

        while (opModeIsActive() && jewelTracker.getLeftColor() == JewelColor.UNKNOWN) {
            sleep(10);
        }

        boolean turnLeft = jewelTracker.getLeftColor().getAllianceColor() != allianceColor;
        double turnAngle = turnLeft ? Math.toRadians(30) : -Math.toRadians(30);
        Path jewelTurn = new Path(Arrays.asList(
                new PointTurn(balancingStonePose, turnAngle),
                new PointTurn(new Pose2d(balancingStonePose.pos(),
                        Angle.norm(balancingStonePose.heading() + turnAngle)), -turnAngle)
        ));

        drive.followPath(jewelTurn);
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        drive.followPath(AutoPaths.makePathToCryptobox(balancingStone, RelicRecoveryVuMark.CENTER));
        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        looper.terminate();
        camera.close();
    }
}
