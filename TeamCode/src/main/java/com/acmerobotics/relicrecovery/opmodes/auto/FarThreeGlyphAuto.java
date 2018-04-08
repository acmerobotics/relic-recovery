package com.acmerobotics.relicrecovery.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.HashMap;
import java.util.Map;

@Autonomous
public class FarThreeGlyphAuto extends AutoOpMode {
    private UltrasonicLocalizer ultrasonicLocalizer;
    private BalancingStone stone;
    private Cryptobox crypto;

    @Override
    protected void setup() {
        stone = robot.config.getBalancingStone();
        crypto = stone.getCryptobox();

        if (stone == BalancingStone.NEAR_BLUE || stone == BalancingStone.NEAR_RED) {
            telemetry.log().add("Invalid balancing stone: " + stone + "!");
            telemetry.update();

            robot.sleep(1);

            requestOpModeStop();

            return;
        }

        ultrasonicLocalizer = new UltrasonicLocalizer(robot.drive);
        robot.addListener(() -> {
            robot.dashboard.getTelemetry().addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
        });
        robot.drive.setLocalizer(ultrasonicLocalizer);
        robot.drive.setEstimatedPosition(stone.getPosition());
    }

    @SuppressLint("DefaultLocale")
    @Override
    protected void run() {
        double startTime = TimestampedData.getCurrentTime();

        int yMultiplier = crypto.getAllianceColor() == AllianceColor.BLUE ? -1 : 1;

        // jewel logic here
        RelicRecoveryVuMark vuMark = vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION = new HashMap<>();
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.LEFT,
                robot.config.getAllianceColor() == AllianceColor.BLUE ? RelicRecoveryVuMark.RIGHT : RelicRecoveryVuMark.CENTER);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.CENTER,
                robot.config.getAllianceColor() == AllianceColor.BLUE ? RelicRecoveryVuMark.RIGHT : RelicRecoveryVuMark.LEFT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.RIGHT,
                robot.config.getAllianceColor() == AllianceColor.BLUE ? RelicRecoveryVuMark.CENTER : RelicRecoveryVuMark.LEFT);

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

        RelicRecoveryVuMark firstColumn = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d biasedFirstColumnPosition = firstColumnPosition.added(new Vector2d(0, -yMultiplier * LATERAL_BIAS));
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);
        Vector2d biasedSecondColumnPosition = secondColumnPosition.added(new Vector2d(0, yMultiplier * LATERAL_BIAS));

        Trajectory stoneToCrypto = new TrajectoryBuilder(stonePose)
                .lineTo(new Vector2d(-44, stonePose.y()))
                .turn(robot.config.getAllianceColor() == AllianceColor.BLUE ? Math.PI : 0)
                .lineTo(new Vector2d(-44, biasedFirstColumnPosition.y()))
                .waitFor(0.25)
                .build();
        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.followTrajectory(stoneToCrypto);

        robot.drive.extendProximitySwivel();
        robot.drive.extendUltrasonicSwivel();

        robot.sleep(0.5);
        raiseArmAndSlapper();
        robot.drive.waitForTrajectoryFollower();

        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.waitOneFullCycle();
        ultrasonicLocalizer.disableUltrasonicFeedback();

        Vector2d estimatedPosition = robot.drive.getEstimatedPosition();
        Trajectory cryptoApproach1 = new TrajectoryBuilder(new Pose2d(estimatedPosition, stoneToCrypto.end().heading()))
                .lineTo(new Vector2d(-56, biasedFirstColumnPosition.y()))
                .waitFor(0.5)
                .build();

        robot.drive.followTrajectory(cryptoApproach1);
        robot.drive.waitForTrajectoryFollower();

        robot.drive.retractUltrasonicSwivel();

        robot.drive.enableHeadingCorrection(cryptoApproach1.end().heading());
        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();
        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(-56, firstColumnPosition.y()));

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        Trajectory cryptoToPit = new TrajectoryBuilder(new Pose2d(-56, firstColumnPosition.y(), cryptoApproach1.end().heading()))
                .lineTo(new Vector2d(-44, firstColumnPosition.y()))
                .lineTo(new Vector2d(-44, yMultiplier * 16))
                .turn(-Math.PI / 4 * yMultiplier)
                .lineTo(new Vector2d(-12, yMultiplier * 16))
                .forward(12)
                .back(12)
                .build();
        robot.drive.followTrajectory(cryptoToPit);
        robot.sleep(0.2 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.autoIntake();
        robot.drive.waitForTrajectoryFollower();

        Trajectory pitToCrypto = new TrajectoryBuilder(cryptoToPit.end())
                .turn(Math.PI / 4 * yMultiplier)
//                .lineTo(new Vector2d(-44, yMultiplier * 16))
                .lineTo(new Vector2d(-44, biasedSecondColumnPosition.y()))
                .waitFor(0.25)
                .build();
        robot.drive.followTrajectory(pitToCrypto);
        robot.sleep(0.5 * pitToCrypto.duration());
        robot.intake.setIntakePower(1);
        robot.drive.extendUltrasonicSwivel();
        robot.drive.extendProximitySwivel();
        robot.drive.waitForTrajectoryFollower();
        robot.intake.setIntakePower(0);

        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.waitOneFullCycle();
        ultrasonicLocalizer.disableUltrasonicFeedback();

        estimatedPosition = robot.drive.getEstimatedPosition();
        Trajectory cryptoApproach2 = new TrajectoryBuilder(new Pose2d(estimatedPosition, pitToCrypto.end().heading()))
                .lineTo(new Vector2d(-56, biasedSecondColumnPosition.y()))
                .waitFor(0.5)
                .build();

        robot.drive.followTrajectory(cryptoApproach2);
        robot.drive.waitForTrajectoryFollower();
        robot.drive.retractUltrasonicSwivel();

        double elapsedTime = TimestampedData.getCurrentTime() - startTime;
        if (elapsedTime < 28) {
            robot.drive.enableHeadingCorrection(cryptoApproach2.end().heading());
            robot.drive.alignWithColumn(robot.config.getAllianceColor());
            robot.drive.waitForColumnAlign();
            robot.drive.disableHeadingCorrection();
            robot.drive.setEstimatedPosition(new Vector2d(-56, secondColumnPosition.y()));

            robot.drive.retractProximitySwivel();
            robot.dumpBed.dump();
            robot.sleep(0.5);

            robot.drive.followTrajectory(new TrajectoryBuilder(cryptoApproach2.end())
                    .forward(6)
                    .build());
            robot.drive.waitForTrajectoryFollower();
            robot.dumpBed.retract();
        } else {
            robot.drive.retractProximitySwivel();
            robot.drive.followTrajectory(new TrajectoryBuilder(cryptoApproach2.end())
                    .forward(6)
                    .build());
            robot.drive.waitForTrajectoryFollower();
        }

        robot.waitOneFullCycle();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
