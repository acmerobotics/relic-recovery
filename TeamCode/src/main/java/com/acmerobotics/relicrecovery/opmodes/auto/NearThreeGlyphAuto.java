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
import com.acmerobotics.relicrecovery.subsystems.Intake;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.HashMap;
import java.util.Map;

@Autonomous(name = "3 Glyph Auto (Near)")
public class NearThreeGlyphAuto extends AutoOpMode {
    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION = new HashMap<>();
    static {
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.CENTER);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.LEFT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.RIGHT, RelicRecoveryVuMark.CENTER);
    }

    private UltrasonicLocalizer ultrasonicLocalizer;
    private BalancingStone stone;
    private Cryptobox crypto;

    @Override
    protected void setup() {
        stone = robot.config.getBalancingStone();
        crypto = stone.getCryptobox();

        if (stone == BalancingStone.FAR_BLUE || stone == BalancingStone.FAR_RED) {
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

        RelicRecoveryVuMark firstColumn = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);

        Trajectory stoneToCrypto = new TrajectoryBuilder(stonePose)
                .lineTo(new Vector2d(firstColumnPosition.x(), stonePose.y()))
                .turn(-Math.PI / 2)
//                .lineTo(new Vector2d(firstColumnPosition.x(), yMultiplier * 57))
                .build();

        Pose2d cryptoPose = stoneToCrypto.end();
        Trajectory cryptoToPit = new TrajectoryBuilder(cryptoPose)
                .lineTo(new Vector2d(cryptoPose.x(), yMultiplier * 48))
                .turn(yMultiplier * Math.PI / 4)
                .lineTo(new Vector2d(cryptoPose.x(), yMultiplier * 12))
                .build();

        Pose2d pitPose = cryptoToPit.end();
        Trajectory pitToCrypto = new TrajectoryBuilder(pitPose)
                .turn(yMultiplier * -Math.PI / 4)
                .lineTo(new Vector2d(secondColumnPosition.x(), yMultiplier * 36))
                .build();

        robot.drive.extendProximitySwivel();
        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.followTrajectory(stoneToCrypto);
        robot.sleep(0.5);
        raiseArmAndSlapper();
        robot.drive.waitForTrajectoryFollower();

        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();

        robot.drive.setEstimatedPosition(stoneToCrypto.end().pos());

        robot.dumpBed.dump();
        robot.sleep(1);

        robot.drive.followTrajectory(cryptoToPit);
        robot.sleep(0.2 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.autoIntake();
        robot.drive.waitForTrajectoryFollower();

        if (robot.intake.getMode() == Intake.Mode.AUTO) {
            // we still didn't get enough glyphs; let's forage a little more
            robot.drive.followTrajectory(new TrajectoryBuilder(cryptoToPit.end())
                    .forward(12 * Math.sqrt(2))
                    .back(12 * Math.sqrt(2))
                    .build());
            robot.drive.waitForTrajectoryFollower();
        }

        robot.drive.followTrajectory(pitToCrypto);
        robot.sleep(0.2 * pitToCrypto.duration());
        robot.drive.extendUltrasonicSwivel();
        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.75);
        robot.intake.setIntakePower(1);
        robot.drive.waitForTrajectoryFollower();
        robot.intake.setIntakePower(0);

        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        Trajectory finalApproach = new TrajectoryBuilder(robot.drive.getEstimatedPose())
                .lineTo(new Vector2d(secondColumnPosition.x(), yMultiplier * 57))
                .build();

        robot.drive.extendProximitySwivel();
        robot.drive.followTrajectory(finalApproach);
        robot.drive.waitForTrajectoryFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();

        robot.drive.retractUltrasonicSwivel();

        robot.drive.enableHeadingCorrection(yMultiplier * -Math.PI / 2);
        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();
        robot.drive.disableHeadingCorrection();

        robot.drive.setEstimatedPosition(finalApproach.end().pos());

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(1);
        robot.drive.followTrajectory(new TrajectoryBuilder(finalApproach.end())
                .forward(6)
                .build());
        robot.drive.waitForTrajectoryFollower();
        robot.dumpBed.retract();
        robot.sleep(0.5);

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
