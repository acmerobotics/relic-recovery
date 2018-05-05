package com.acmerobotics.relicrecovery.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.LoggingUtil;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.localization.DeadReckoningLocalizer;
import com.acmerobotics.relicrecovery.localization.Localizer;
import com.acmerobotics.relicrecovery.localization.TrackingOmniLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.library.path.Trajectory;
import com.acmerobotics.library.path.TrajectoryBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.io.File;
import java.util.HashMap;
import java.util.Map;

@Disabled
@Autonomous
public class SplineNearSixGlyphAutoNoUltrasonic extends AutoOpMode {
    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION1 = new HashMap<>();
    static {
        COLUMN_TRANSITION1.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION1.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION1.put(RelicRecoveryVuMark.RIGHT, RelicRecoveryVuMark.LEFT);
    }
    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION2 = new HashMap<>();
    static {
        COLUMN_TRANSITION2.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.CENTER);
        COLUMN_TRANSITION2.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.LEFT);
        COLUMN_TRANSITION2.put(RelicRecoveryVuMark.RIGHT, RelicRecoveryVuMark.CENTER);
    }

    private BalancingStone stone;
    private Cryptobox crypto;

    @Override
    protected void setup() {
        stone = robot.config.getBalancingStone();
        crypto = stone.getCryptobox();
//        robot.drive.setLocalizer(new TrackingOmniLocalizer(robot.drive));
        CSVLoggingTelemetry positionComparison = new CSVLoggingTelemetry(new File(LoggingUtil.getLogRoot(this),
                "PositionComparison-" + System.currentTimeMillis() + ".csv"));
        DeadReckoningLocalizer deadReckoningLocalizer = new DeadReckoningLocalizer(robot.drive);
        TrackingOmniLocalizer trackingOmniLocalizer = new TrackingOmniLocalizer(robot.drive);
        robot.drive.setLocalizer(new Localizer() {
            @Override
            public Vector2d update() {
                Vector2d deadReckoningPosition = deadReckoningLocalizer.update();
                Vector2d trackingOmniPosition = trackingOmniLocalizer.update();

                positionComparison.addData("deadReckoningX", deadReckoningPosition.x());
                positionComparison.addData("deadReckoningY", deadReckoningPosition.y());
                positionComparison.addData("trackingOmniX", trackingOmniPosition.x());
                positionComparison.addData("trackingOmniY", trackingOmniPosition.y());
                positionComparison.update();

                return deadReckoningPosition;
            }

            @Override
            public void setEstimatedPosition(Vector2d position) {
                deadReckoningLocalizer.setEstimatedPosition(position);
                trackingOmniLocalizer.setEstimatedPosition(position);
            }
        });
        robot.drive.setEstimatedPosition(stone.getPosition());
        robot.drive.setVelocityPIDCoefficients(MecanumDrive.SLOW_VELOCITY_PID);
    }

    @SuppressLint("DefaultLocale")
    @Override
    protected void run() {
        double startTime = TimestampedData.getCurrentTime();

        int yMultiplier = (crypto.getAllianceColor() == AllianceColor.BLUE) ? -1 : 1;

        RelicRecoveryVuMark vuMark = vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        lowerArmAndSlapper();

        boolean removeLeft = robot.config.getAllianceColor() == jewelPosition.rightColor();

        if (removeLeft && robot.config.getAllianceColor() == AllianceColor.BLUE) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
            robot.sleep(0.75);
            raiseArmAndSlapper();
        } else if (!removeLeft && robot.config.getAllianceColor() == AllianceColor.RED) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
            robot.sleep(0.75);
            raiseArmAndSlapper();
        } else {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.PARALLEL);
            robot.sleep(0.75);
        }

        RelicRecoveryVuMark firstColumn = (vuMark == RelicRecoveryVuMark.UNKNOWN) ? RelicRecoveryVuMark.LEFT : vuMark;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION1.get(firstColumn);
        RelicRecoveryVuMark thirdColumn = COLUMN_TRANSITION2.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);
        Vector2d thirdColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, thirdColumn);

        Trajectory stoneToPit = robot.drive.trajectoryBuilder(stonePose)
                .beginComposite()
                .lineTo(new Vector2d(30, stonePose.y()))
                .splineThrough(new Pose2d(0, yMultiplier * 12, -yMultiplier * 3 * Math.PI / 4))
                .closeComposite()
                .build();
        robot.drive.setEstimatedPose(stoneToPit.start());
        robot.drive.followTrajectory(stoneToPit);
        robot.sleep(0.5);
        raiseArmAndSlapper();
        robot.sleep(0.5);
        robot.intake.autoIntake();
        robot.sleep(0.75 * stoneToPit.duration() - 1);
        robot.drive.setVelocityPIDCoefficients(MecanumDrive.NORMAL_VELOCITY_PID);
        robot.drive.waitForTrajectoryFollower();

        Trajectory pitToCrypto1 = robot.drive.trajectoryBuilder(stoneToPit.end())
                .beginComposite()
                .splineThrough(new Pose2d(firstColumnPosition.x(), yMultiplier * 40, -yMultiplier * Math.PI / 2))
                .lineTo(new Vector2d(firstColumnPosition.x(), yMultiplier * 56))
                .closeComposite()
                .waitFor(0.5)
                .build();
        robot.drive.followTrajectory(pitToCrypto1);

        robot.drive.extendProximitySwivel();

        robot.sleep(0.5 * pitToCrypto1.duration());
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto1.duration());
        robot.intake.setIntakePower(0);
        robot.drive.waitForTrajectoryFollower();

        robot.drive.enableHeadingCorrection(-yMultiplier * Math.PI / 2);

        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(pitToCrypto1.end().pos());

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        Trajectory cryptoToPit2 = robot.drive.trajectoryBuilder(pitToCrypto1.end())
                .splineThrough(new Pose2d(24, yMultiplier * 12, -yMultiplier * Math.PI / 4))
                .build();
        robot.drive.followTrajectory(cryptoToPit2);

        robot.sleep(0.25 * cryptoToPit2.duration());

        robot.dumpBed.retract();
        robot.intake.autoIntake();

        robot.drive.waitForTrajectoryFollower();

        Trajectory pitToCrypto2 = robot.drive.trajectoryBuilder(cryptoToPit2.end())
                .beginComposite()
                .splineThrough(new Pose2d(secondColumnPosition.x(), yMultiplier * 40, -yMultiplier * Math.PI / 2))
                .lineTo(new Vector2d(secondColumnPosition.x(), yMultiplier * 56))
                .closeComposite()
                .waitFor(0.5)
                .build();
        robot.drive.followTrajectory(pitToCrypto2);

        robot.drive.extendProximitySwivel();

        robot.sleep(0.5 * pitToCrypto2.duration());
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto2.duration());
        robot.intake.setIntakePower(0);
        robot.drive.waitForTrajectoryFollower();

        robot.drive.enableHeadingCorrection(-yMultiplier * Math.PI / 2);

        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(pitToCrypto2.end().pos());

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        Trajectory cryptoToPit3 = robot.drive.trajectoryBuilder(pitToCrypto2.end())
                .splineThrough(new Pose2d(16, 0, -yMultiplier * 3 * Math.PI / 8))
                .build();
        robot.drive.followTrajectory(cryptoToPit3);

        robot.sleep(0.25 * cryptoToPit3.duration());

        robot.dumpBed.retract();
        robot.intake.autoIntake();

        robot.drive.waitForTrajectoryFollower();

        Trajectory pitToCrypto3 = robot.drive.trajectoryBuilder(cryptoToPit3.end())
                .beginComposite()
                .splineThrough(new Pose2d(thirdColumnPosition.x(), yMultiplier * 40, -yMultiplier * Math.PI / 2))
                .lineTo(new Vector2d(thirdColumnPosition.x(), yMultiplier * 56))
                .closeComposite()
                .waitFor(0.5)
                .build();
        robot.drive.followTrajectory(pitToCrypto3);

        robot.drive.extendProximitySwivel(); //extend the proximity sensor

        robot.sleep(0.5 * pitToCrypto3.duration());
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto3.duration());
        robot.intake.setIntakePower(0);
        robot.drive.waitForTrajectoryFollower();

        robot.drive.enableHeadingCorrection(-yMultiplier * Math.PI / 2);

        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(pitToCrypto3.end().pos());

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(pitToCrypto3.end())
                .forward(8)
                .build());
        robot.drive.waitForTrajectoryFollower();

        robot.dumpBed.retract();

        robot.waitOneFullCycle();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
