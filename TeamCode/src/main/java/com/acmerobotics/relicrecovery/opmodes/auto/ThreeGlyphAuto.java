package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static com.acmerobotics.relicrecovery.opmodes.AutoPaths.CRYPTO_COL_WIDTH;

@Config
@Autonomous(name = "3 Glyph Auto")
public class ThreeGlyphAuto extends AutoOpMode {
    public static RelicRecoveryVuMark VUMARK = RelicRecoveryVuMark.CENTER;

    private UltrasonicLocalizer ultrasonicLocalizer;

    @Override
    protected void setup() {
        ultrasonicLocalizer = new UltrasonicLocalizer(robot.drive);
        robot.addListener(() -> {
            robot.dashboard.getTelemetry().addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
        });
        robot.drive.setLocalizer(ultrasonicLocalizer);
        robot.drive.setEstimatedPosition(BalancingStone.NEAR_BLUE.getPosition());
    }

    @Override
    protected void run() {
        double startTime = TimestampedData.getCurrentTime();

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);

        Path stoneToCrypto = null;
        switch (VUMARK) {
            case LEFT:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -56))
                        .build();
                break;
            case CENTER:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(12, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(12, -56))
                        .build();
                break;
            case RIGHT:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(12 - CRYPTO_COL_WIDTH, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(12 - CRYPTO_COL_WIDTH, -58))
                        .build();
                break;
        }

        Pose2d cryptoPose = stoneToCrypto.end();
        Path cryptoToPit = new PathBuilder(cryptoPose)
                .lineTo(new Vector2d(cryptoPose.x(), -48))
                .turn(Math.PI / 4)
                .lineTo(new Vector2d(cryptoPose.x(), -12))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(cryptoPose.x(), -36))
                .build();

        Pose2d pitPose = cryptoToPit.end();
        Path pitToCrypto = null;
        UltrasonicLocalizer.UltrasonicTarget ultrasonicTarget = null;
        switch (VUMARK) {
            case LEFT:
                pitToCrypto = new PathBuilder(pitPose)
                        .lineTo(new Vector2d(12, -36))
                        .lineTo(new Vector2d(12, -56))
                        .build();
                ultrasonicTarget = UltrasonicLocalizer.UltrasonicTarget.WALL;
                break;
            case CENTER:
                pitToCrypto = new PathBuilder(pitPose)
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -36))
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -56))
                        .build();
                ultrasonicTarget = UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN;
                break;
            case RIGHT:
                pitToCrypto = new PathBuilder(pitPose)
                        .lineTo(new Vector2d(12, -36))
                        .lineTo(new Vector2d(12, -56))
                        .build();
                ultrasonicTarget = UltrasonicLocalizer.UltrasonicTarget.WALL;
                break;
        }

        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.followPath(stoneToCrypto);
        robot.drive.waitForPathFollower();

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();
        robot.dumpBed.dump();
        robot.sleep(0.5);

        robot.drive.followPath(cryptoToPit);
        robot.sleep(0.2 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.setIntakePower(1);
        robot.sleep(0.6 * cryptoToPit.duration());
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.drive.waitForPathFollower();
        robot.intake.setIntakePower(-1);

        ultrasonicLocalizer.setTarget(ultrasonicTarget);
        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.drive.extendSideSwivel();
        robot.drive.followPath(pitToCrypto);
        robot.drive.waitForPathFollower();

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();
        robot.drive.retractSideSwivel();
        robot.dumpBed.dump();
        robot.sleep(0.5);
        robot.drive.followPath(new PathBuilder(pitToCrypto.end())
                .back(6)
                .build());
        robot.drive.waitForPathFollower();
        robot.dumpBed.retract();
        robot.sleep(0.5);

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();
    }
}
