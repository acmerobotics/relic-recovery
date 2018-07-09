package com.acmerobotics.relicrecovery.opmodes.auto;

import android.annotation.SuppressLint;
import android.util.TimingLogger;

import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.acmerobotics.splinelib.Pose2d;
import com.acmerobotics.splinelib.Vector2d;
import com.acmerobotics.splinelib.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.HashMap;
import java.util.Map;

@Disabled
@Autonomous
public class SplineFarFourGlyphAuto extends AutoOpMode {
    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION1 = new HashMap<>();
    static {
        COLUMN_TRANSITION1.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION1.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION1.put(RelicRecoveryVuMark.RIGHT, RelicRecoveryVuMark.CENTER);
    }

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
        TimingLogger timings = new TimingLogger("Auto", "splineFarFourGlyph");
        double startTime = TimestampedData.getCurrentTime();

        int yMultiplier = (crypto.getAllianceColor() == AllianceColor.BLUE) ? -1 : 1;

        RelicRecoveryVuMark vuMark = vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        timings.addSplit("vuMark");

        lowerArmAndSlapper();

        boolean removeLeft = robot.config.getAllianceColor() == jewelPosition.rightColor();

        if (removeLeft) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
        } else {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
        }
        robot.sleep(0.75);
        raiseArmAndSlapper();

        timings.addSplit("jewel");

        RelicRecoveryVuMark firstColumn = (vuMark == RelicRecoveryVuMark.UNKNOWN) ? RelicRecoveryVuMark.LEFT : vuMark;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION1.get(firstColumn);

        Pose2d stonePose = AutoPaths.getBalancingStonePose(stone).plus(new Pose2d(0, yMultiplier * AutoPaths.STONE_CORRECTION, 0));
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);

        Trajectory stoneToPit = robot.drive.trajectoryBuilder(stonePose)
                .turnTo(-yMultiplier * Math.PI / 2)
                .beginComposite()
                .lineTo(new Vector2d(stonePose.x(),  yMultiplier * 36))
                .splineTo(new Pose2d(-8, -8, -yMultiplier * Math.PI / 4))
                .closeComposite()
                .build();

        timings.addSplit("stoneToPit gen");

        robot.drive.setEstimatedPose(stoneToPit.start());
        robot.drive.followTrajectory(stoneToPit);
        robot.sleep(1);
        robot.intake.autoIntake();
        robot.drive.waitForTrajectoryFollower();

        timings.addSplit("stoneToPit");

        Trajectory pitToCrypto1 = robot.drive.trajectoryBuilder(stoneToPit.end())
                .splineTo(new Pose2d(-24, yMultiplier * 12, 0))
                .splineTo(new Pose2d(-56, firstColumnPosition.y(), 0))
                .waitFor(0.5)
                .build();

        timings.addSplit("pitToCrypto1 gen");

        robot.drive.followTrajectory(pitToCrypto1);

        robot.drive.extendProximitySwivel();

        robot.sleep(0.5 * pitToCrypto1.duration());
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto1.duration());
        robot.intake.setIntakePower(0);
        robot.drive.waitForTrajectoryFollower();

        timings.addSplit("pitToCrypto1");

        robot.drive.enableHeadingCorrection(0);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(pitToCrypto1.end().pos());

        timings.addSplit("columnAlign1");

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        timings.addSplit("dump1");

        Trajectory cryptoToPit2 = robot.drive.trajectoryBuilder(pitToCrypto1.end())
                .splineTo(new Pose2d(-24, yMultiplier * 12, 0))
                .splineTo(new Pose2d(0, 0, -yMultiplier * Math.PI / 4))
                .build();

        timings.addSplit("cryptoToPit2 gen");

        robot.drive.followTrajectory(cryptoToPit2);

        robot.sleep(0.25 * cryptoToPit2.duration());

        robot.dumpBed.retract();
        robot.intake.autoIntake();

        robot.drive.waitForTrajectoryFollower();

        timings.addSplit("cryptoToPit2");

        Trajectory pitToCrypto2 = robot.drive.trajectoryBuilder(cryptoToPit2.end())
                .splineTo(new Pose2d(-16, yMultiplier * 12, 0))
                .splineTo(new Pose2d(-40, secondColumnPosition.y(), 0))
                .waitFor(0.25)
                .build();

        timings.addSplit("pitToCrypto2 gen");

        robot.drive.followTrajectory(pitToCrypto2);

        robot.drive.extendUltrasonicSwivel();
        robot.drive.extendProximitySwivel();

        robot.sleep(0.5 * pitToCrypto2.duration());
        robot.intake.setIntakePower(1);
        robot.sleep(0.3 * pitToCrypto2.duration());
        robot.intake.setIntakePower(0);
        robot.drive.waitForTrajectoryFollower();

        timings.addSplit("pitToCrypto2");

        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.waitOneFullCycle();
        ultrasonicLocalizer.disableUltrasonicFeedback();

        Trajectory cryptoApproach2 = robot.drive.trajectoryBuilder(new Pose2d(robot.drive.getEstimatedPosition(), pitToCrypto2.end().heading()))
                .lineTo(new Vector2d(-56, secondColumnPosition.y()))
                .waitFor(0.5)
                .build();

        timings.addSplit("cryptoApproach2 gen");

        robot.drive.followTrajectory(cryptoApproach2);
        robot.drive.waitForTrajectoryFollower();

        timings.addSplit("cryptoApproach2");

        robot.drive.retractUltrasonicSwivel();
        robot.drive.enableHeadingCorrection(0);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(cryptoApproach2.end().pos());

        timings.addSplit("columnAlign2");

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        timings.addSplit("dump2");

        robot.drive.followTrajectory(robot.drive.trajectoryBuilder(cryptoApproach2.end())
                .forward(8)
                .build());

        timings.addSplit("return gen");

        robot.drive.waitForTrajectoryFollower();

        timings.addSplit("return");

        robot.dumpBed.retract();

        robot.waitOneFullCycle();

        timings.dumpToLog();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
