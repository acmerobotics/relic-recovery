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
import com.acmerobotics.relicrecovery.subsystems.Intake;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.HashMap;
import java.util.Map;

@Config
@Autonomous(name = "3 Glyph Auto (Near)")
public class NearThreeGlyphAuto extends AutoOpMode {
    public static RelicRecoveryVuMark VUMARK = RelicRecoveryVuMark.CENTER;

    public static final Map<RelicRecoveryVuMark, RelicRecoveryVuMark> COLUMN_TRANSITION = new HashMap<>();
    static {
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.LEFT, RelicRecoveryVuMark.CENTER);
        COLUMN_TRANSITION.put(RelicRecoveryVuMark.CENTER, RelicRecoveryVuMark.LEFT);
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

        robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);

        int yMultiplier = crypto.getAllianceColor() == AllianceColor.BLUE ? -1 : 1;

        // jewel logic here
        RelicRecoveryVuMark firstColumn = VUMARK;
        RelicRecoveryVuMark secondColumn = COLUMN_TRANSITION.get(firstColumn);

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(stone);
        Vector2d firstColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, firstColumn);
        Vector2d secondColumnPosition = AutoPaths.getCryptoboxColumnPosition(crypto, secondColumn);

        Path stoneToCrypto = new PathBuilder(stonePose)
                .lineTo(new Vector2d(firstColumnPosition.x(), stonePose.y()))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(firstColumnPosition.x(), yMultiplier * 56))
                .build();

        Pose2d cryptoPose = stoneToCrypto.end();
        Path cryptoToPit = new PathBuilder(cryptoPose)
                .lineTo(new Vector2d(cryptoPose.x(), yMultiplier * 48))
                .turn(Math.PI / 4)
                .lineTo(new Vector2d(cryptoPose.x(), yMultiplier * 12))
                .build();

        Pose2d pitPose = cryptoToPit.end();
        Path pitToCrypto = new PathBuilder(pitPose)
//                .lineTo(new Vector2d(cryptoPose.x(), yMultiplier * 36))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(secondColumnPosition.x(), yMultiplier * 36))
                .build();

        Path finalApproach = new PathBuilder(pitToCrypto.end())
                .lineTo(new Vector2d(secondColumnPosition.x(), yMultiplier * 56))
                .build();
        UltrasonicLocalizer.UltrasonicTarget ultrasonicTarget = ULTRASONIC_TARGETS.get(secondColumn);

        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.extendSideSwivel();
        robot.drive.followPath(stoneToCrypto);
        robot.drive.waitForPathFollower();

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.setEstimatedPosition(stoneToCrypto.end().pos());

        robot.dumpBed.dump();
        robot.sleep(1);

        robot.drive.followPath(cryptoToPit);
        robot.sleep(0.2 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.autoIntake();
        robot.drive.waitForPathFollower();

        if (robot.intake.getMode() == Intake.Mode.AUTO) {
            // we still didn't get enough glyphs; let's forage a little more
            robot.drive.followPath(new PathBuilder(cryptoToPit.end())
                    .forward(12 * Math.sqrt(2))
                    .back(12 * Math.sqrt(2))
                    .build());
            robot.drive.waitForPathFollower();
        }

        robot.drive.followPath(pitToCrypto);
        robot.sleep(0.2 * pitToCrypto.duration());
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.75);
        robot.intake.setIntakePower(1);
        robot.drive.waitForPathFollower();
        robot.intake.setIntakePower(0);

        ultrasonicLocalizer.setTarget(ultrasonicTarget);
        ultrasonicLocalizer.enableUltrasonicFeedback();

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
