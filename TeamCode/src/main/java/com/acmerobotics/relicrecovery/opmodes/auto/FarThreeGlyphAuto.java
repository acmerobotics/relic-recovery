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
import com.acmerobotics.relicrecovery.path2.PathBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name = "3 Glyph Auto (Far)")
public class FarThreeGlyphAuto extends AutoOpMode {
    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION = new HashMap<>();
    static {
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.RIGHT);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.RIGHT, RelicRecoveryVuMark.CENTER);
    }

    public static final Map<RelicRecoveryVuMark, UltrasonicLocalizer.UltrasonicTarget> ULTRASONIC_TARGETS = new HashMap<>();
    static {
        ULTRASONIC_TARGETS.put(RelicRecoveryVuMark.LEFT, UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ULTRASONIC_TARGETS.put(RelicRecoveryVuMark.CENTER, UltrasonicLocalizer.UltrasonicTarget.WALL);
        ULTRASONIC_TARGETS.put(RelicRecoveryVuMark.RIGHT, UltrasonicLocalizer.UltrasonicTarget.WALL);
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
        double startTime = TimestampedData.getCurrentTime();

        robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);

        int yMultiplier = crypto.getAllianceColor() == AllianceColor.BLUE ? -1 : 1;

        // jewel logic here
        RelicRecoveryVuMark vuMark = vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        robot.jewelSlapper.lowerArmAndSlapper();

        robot.sleep(0.75);

        boolean removeLeft = robot.config.getAllianceColor() == jewelPosition.rightColor();

        if (removeLeft && robot.config.getAllianceColor() == AllianceColor.BLUE) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
            robot.sleep(0.5);
            robot.jewelSlapper.stowArmAndSlapper();
            robot.sleep(0.75);
        } else if (!removeLeft && robot.config.getAllianceColor() == AllianceColor.RED) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
            robot.sleep(0.5);
            robot.jewelSlapper.stowArmAndSlapper();
            robot.sleep(0.75);
        } else {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.PARALLEL);
            robot.sleep(0.5);
        }

        RelicRecoveryVuMark firstColumn = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);

        Path stoneToCrypto = new PathBuilder(stonePose)
                .lineTo(new Vector2d(-48, stonePose.y()))
                .turn(robot.config.getAllianceColor() == AllianceColor.BLUE ? Math.PI : 0)
                .lineTo(new Vector2d(-48, firstColumnPosition.y()))
                .lineTo(new Vector2d(-57, firstColumnPosition.y()))
                .build();

        Pose2d cryptoPose = stoneToCrypto.end();
        Path cryptoToPit = new PathBuilder(cryptoPose)
                .lineTo(new Vector2d(-48, firstColumnPosition.y()))
                .lineTo(new Vector2d(-48, yMultiplier * 12))
                .lineTo(new Vector2d(0, yMultiplier * 12))
                .build();

        Pose2d pitPose = cryptoToPit.end();
        Path pitToCrypto = new PathBuilder(pitPose)
                .lineTo(new Vector2d(-48, yMultiplier * 12))
                .lineTo(new Vector2d(-48, secondColumnPosition.y()))
                .build();

        UltrasonicLocalizer.UltrasonicTarget ultrasonicTarget = ULTRASONIC_TARGETS.get(secondColumn);

        double headingP = MecanumDrive.HEADING_PID.p;
        MecanumDrive.HEADING_PID.p = 0;
        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.followPath(stoneToCrypto);
        robot.sleep(0.5);
        robot.jewelSlapper.stowArmAndSlapper();
        robot.drive.waitForPathFollower();
        MecanumDrive.HEADING_PID.p = headingP;

//        robot.drive.alignWithColumn();
//        robot.drive.waitForColumnAlign();
//
//        robot.drive.setEstimatedPosition(stoneToCrypto.end().pos());

        robot.dumpBed.dump();
        robot.sleep(1);

        robot.drive.followPath(cryptoToPit);
        robot.sleep(0.2 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.autoIntake();
        robot.drive.waitForPathFollower();

        robot.drive.followPath(pitToCrypto);
        robot.sleep(0.2 * pitToCrypto.duration());
        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.75);
        robot.intake.setIntakePower(1);
        robot.sleep(0.6 * pitToCrypto.duration() - 0.75);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.drive.waitForPathFollower();
        robot.intake.setIntakePower(0);

        ultrasonicLocalizer.setTarget(ultrasonicTarget);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        Path finalApproach = new PathBuilder(robot.drive.getEstimatedPose())
                .lineTo(new Vector2d(-57, secondColumnPosition.y()))
                .build();

        robot.drive.extendSideSwivel();
        robot.drive.followPath(finalApproach);
        robot.drive.waitForPathFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();

        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.setEstimatedPosition(finalApproach.end().pos());

        robot.drive.retractSideSwivel();
        robot.dumpBed.dump();
        robot.sleep(1);
        robot.drive.followPath(new PathBuilder(finalApproach.end())
                .forward(6)
                .build());
        robot.drive.waitForPathFollower();
        robot.dumpBed.retract();
        robot.sleep(0.5);

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
