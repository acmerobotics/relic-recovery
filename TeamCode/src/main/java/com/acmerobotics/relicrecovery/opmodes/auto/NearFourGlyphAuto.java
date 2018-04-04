package com.acmerobotics.relicrecovery.opmodes.auto;

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

@Autonomous
public class NearFourGlyphAuto extends AutoOpMode {
    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION = new HashMap<>();
    static {
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.RIGHT, RelicRecoveryVuMark.LEFT);
    }

    private UltrasonicLocalizer ultrasonicLocalizer;
    private BalancingStone stone;
    private Cryptobox crypto;

    @Override
    protected void setup() {
        stone = robot.config.getBalancingStone();
        crypto = stone.getCryptobox();

        ultrasonicLocalizer = new UltrasonicLocalizer(robot.drive);
        robot.addListener(() -> {
            robot.dashboard.getTelemetry().addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
        });
        robot.drive.setLocalizer(ultrasonicLocalizer);
        robot.drive.setEstimatedPosition(stone.getPosition());
    }

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
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);
        Vector2d biasedSecondColumnPosition = secondColumnPosition.added(new Vector2d(-yMultiplier * AutoPaths.VUMARK_MAP.get(secondColumn) * LATERAL_BIAS, 0));

        Trajectory stoneToFloor = new TrajectoryBuilder(stonePose)
                .lineTo(new Vector2d(firstColumnPosition.x(), stonePose.y()))
                .build();
        robot.drive.setEstimatedPose(stoneToFloor.start());
        robot.drive.followTrajectory(stoneToFloor);
        robot.sleep(0.5);
        raiseArmAndSlapper();
        robot.drive.waitForTrajectoryFollower();

        robot.intake.autoIntake();

        Trajectory floorToPit = new TrajectoryBuilder(stoneToFloor.end())
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(firstColumnPosition.x(), yMultiplier * 12))
                .turn(-Math.PI / 4)
                .build();
        robot.drive.followTrajectory(floorToPit);
        robot.drive.waitForTrajectoryFollower();

        Trajectory pitToCrypto1 = new TrajectoryBuilder(floorToPit.end())
                .lineTo(new Vector2d(firstColumnPosition.x(), yMultiplier * 59))
                .build();
        robot.drive.followTrajectory(pitToCrypto1);

        robot.drive.extendUltrasonicSwivel();
        robot.drive.extendProximitySwivel();

        robot.sleep(0.2 * pitToCrypto1.duration());
        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.5);
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto1.duration() - 0.5);
        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.drive.waitForTrajectoryFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.drive.retractUltrasonicSwivel();
        robot.drive.enableHeadingCorrection(-yMultiplier * Math.PI / 2);

        robot.drive.alignWithColumn(robot.config.getAllianceColor());
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(pitToCrypto1.end().pos());

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        Trajectory cryptoToPit2 = new TrajectoryBuilder(pitToCrypto1.end())
                .lineTo(new Vector2d(biasedSecondColumnPosition.x(), yMultiplier * 12))
                .turn(-Math.PI / 4)
                .build();
        robot.drive.followTrajectory(cryptoToPit2);

        robot.sleep(0.25 * cryptoToPit2.duration());

        robot.dumpBed.retract();
        robot.intake.autoIntake();

        robot.drive.waitForTrajectoryFollower();

        if (robot.intake.getMode() == Intake.Mode.AUTO) {
            // we still didn't get enough glyphs; let's forage a little more
            robot.drive.followTrajectory(new TrajectoryBuilder(cryptoToPit2.end())
                    .forward(12)
                    .back(12)
                    .turn(Math.PI / 4)
                    .build());
        } else {
            robot.drive.followTrajectory(new TrajectoryBuilder(cryptoToPit2.end())
                    .turn(Math.PI / 4)
                    .build());
        }
        robot.drive.waitForTrajectoryFollower();

        Trajectory pitToCrypto2 = new TrajectoryBuilder(new Pose2d(cryptoToPit2.end().pos(), -yMultiplier * Math.PI / 2))
                .lineTo(new Vector2d(biasedSecondColumnPosition.x(), yMultiplier * 60))
                .build();

        robot.drive.followTrajectory(pitToCrypto2);
        robot.drive.extendUltrasonicSwivel();
        robot.drive.extendProximitySwivel();

        robot.sleep(0.2 * pitToCrypto2.duration());
        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.5);
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto2.duration() - 0.5);
        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.drive.waitForTrajectoryFollower();
        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.drive.retractUltrasonicSwivel();

        double elapsedTime = TimestampedData.getCurrentTime() - startTime;
        if (elapsedTime < 28) {
            robot.drive.enableHeadingCorrection(-yMultiplier * Math.PI / 2);
            robot.drive.alignWithColumn(robot.config.getAllianceColor());
            robot.drive.waitForColumnAlign();
            robot.drive.disableHeadingCorrection();
            robot.drive.setEstimatedPosition(pitToCrypto2.end().pos());

            robot.drive.retractProximitySwivel();
            robot.dumpBed.dump();
            robot.sleep(0.5);

            robot.drive.followTrajectory(new TrajectoryBuilder(pitToCrypto2.end())
                    .forward(8)
                    .build());
            robot.drive.waitForTrajectoryFollower();

            robot.dumpBed.retract();
        } else {
            robot.drive.followTrajectory(new TrajectoryBuilder(pitToCrypto2.end())
                    .forward(8)
                    .build());
            robot.drive.waitForTrajectoryFollower();
        }

        robot.waitOneFullCycle();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
