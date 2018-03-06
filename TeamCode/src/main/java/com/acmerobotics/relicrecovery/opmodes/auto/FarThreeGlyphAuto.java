package com.acmerobotics.relicrecovery.opmodes.auto;

import android.annotation.SuppressLint;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name = "3 Glyph Auto (Far)")
public class FarThreeGlyphAuto extends AutoOpMode {
    public static RelicRecoveryVuMark VUMARK = RelicRecoveryVuMark.CENTER;

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
        RelicRecoveryVuMark vuMark = VUMARK; // vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION = new HashMap<>();
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.LEFT,
                robot.config.getAllianceColor() == AllianceColor.BLUE ? RelicRecoveryVuMark.RIGHT : RelicRecoveryVuMark.CENTER);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.CENTER,
                robot.config.getAllianceColor() == AllianceColor.BLUE ? RelicRecoveryVuMark.RIGHT : RelicRecoveryVuMark.LEFT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.RIGHT,
                robot.config.getAllianceColor() == AllianceColor.BLUE ? RelicRecoveryVuMark.CENTER : RelicRecoveryVuMark.LEFT);

        robot.jewelSlapper.lowerArmAndSlapper();

        robot.sleep(0.75);

        boolean removeLeft = robot.config.getAllianceColor() == jewelPosition.rightColor();

        if (removeLeft && robot.config.getAllianceColor() == AllianceColor.BLUE) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
            robot.sleep(0.5);
            robot.jewelSlapper.stowArmAndSlapper();
        } else if (!removeLeft && robot.config.getAllianceColor() == AllianceColor.RED) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
            robot.sleep(0.5);
            robot.jewelSlapper.stowArmAndSlapper();
        } else {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.PARALLEL);
            robot.sleep(0.5);
        }

        RelicRecoveryVuMark firstColumn = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d biasedFirstColumnPosition = firstColumnPosition.added(new Vector2d(0, -yMultiplier * LATERAL_BIAS));
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);
        Vector2d biasedSecondColumnPosition = secondColumnPosition.added(new Vector2d(0, yMultiplier * LATERAL_BIAS));

        Path stoneToCrypto = new PathBuilder(stonePose)
                .lineTo(new Vector2d(-48, stonePose.y()))
                .turn(robot.config.getAllianceColor() == AllianceColor.BLUE ? Math.PI : 0)
                .lineTo(new Vector2d(-48, biasedFirstColumnPosition.y()))
                .build();
        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.followPath(stoneToCrypto);

        robot.drive.extendProximitySwivel();
        robot.drive.extendUltrasonicSwivel();

        robot.sleep(0.5);
        robot.jewelSlapper.stowArmAndSlapper();
        robot.drive.waitForPathFollower();

        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        Vector2d estimatedPosition = robot.drive.getEstimatedPosition();
        Path cryptoApproach1 = new PathBuilder(new Pose2d(estimatedPosition, stoneToCrypto.end().heading()))
                .lineTo(new Vector2d(-56, biasedFirstColumnPosition.y()))
                .build();

        robot.drive.followPath(cryptoApproach1);
        robot.drive.waitForPathFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.drive.retractUltrasonicSwivel();

        robot.drive.enableHeadingCorrection(cryptoApproach1.end().heading());
        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();
        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(-56, firstColumnPosition.y()));

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        Path cryptoToPit = new PathBuilder(new Pose2d(-56, firstColumnPosition.y(), cryptoApproach1.end().heading()))
                .lineTo(new Vector2d(-48, firstColumnPosition.y()))
                .lineTo(new Vector2d(-48, yMultiplier * 16))
                .turn(-Math.PI / 4 * yMultiplier)
                .lineTo(new Vector2d(-12, yMultiplier * 16))
                .forward(12)
                .back(12)
                .build();
        robot.drive.followPath(cryptoToPit);
        robot.sleep(0.2 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.autoIntake();
        robot.drive.waitForPathFollower();

        Path pitToCrypto = new PathBuilder(cryptoToPit.end())
                .turn(Math.PI / 4 * yMultiplier)
//                .lineTo(new Vector2d(-48, yMultiplier * 16))
                .lineTo(new Vector2d(-48, biasedSecondColumnPosition.y()))
                .build();
        robot.drive.followPath(pitToCrypto);
        robot.sleep(0.2 * pitToCrypto.duration());
        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.75);
        robot.intake.setIntakePower(1);
        robot.drive.extendUltrasonicSwivel();
        robot.drive.extendProximitySwivel();
        robot.drive.waitForPathFollower();
        robot.intake.setIntakePower(0);

        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        estimatedPosition = robot.drive.getEstimatedPosition();
        Path cryptoApproach2 = new PathBuilder(new Pose2d(estimatedPosition, pitToCrypto.end().heading()))
                .lineTo(new Vector2d(-56, biasedSecondColumnPosition.y()))
                .build();

        robot.drive.followPath(cryptoApproach2);
        robot.drive.waitForPathFollower();
        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.drive.retractUltrasonicSwivel();

        robot.drive.enableHeadingCorrection(cryptoApproach2.end().heading());
        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();
        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(-56, secondColumnPosition.y()));

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        robot.drive.followPath(new PathBuilder(cryptoApproach2.end())
                .forward(6)
                .build());
        robot.drive.waitForPathFollower();
        robot.dumpBed.retract();

        robot.waitOneFullCycle();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
