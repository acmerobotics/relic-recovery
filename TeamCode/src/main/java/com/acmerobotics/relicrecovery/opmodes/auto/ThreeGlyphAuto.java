package com.acmerobotics.relicrecovery.opmodes.auto;

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

@Autonomous(name = "3 Glyph Auto")
public class ThreeGlyphAuto extends AutoOpMode {
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

        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.CENTER;

        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);

        Path stoneToCrypto = null;
        switch (vuMark) {
            case LEFT:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(CRYPTO_COL_WIDTH - 6, -48))
                        .turn(-Math.PI / 4)
                        .lineTo(new Vector2d(CRYPTO_COL_WIDTH + 4, -58))
                        .build();
                break;
            case CENTER:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(16 + 10 / Math.sqrt(3), -48))
                        .turn(-2 * Math.PI / 3)
                        .lineTo(new Vector2d(16, -58))
                        .build();
                break;
            case RIGHT:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(30 - CRYPTO_COL_WIDTH, -48))
                        .turn(-3 * Math.PI / 4)
                        .lineTo(new Vector2d(20 - CRYPTO_COL_WIDTH, -58))
                        .build();
                break;
        }

        UltrasonicLocalizer.UltrasonicTarget ultrasonicTarget = null;
        Path cryptoToPit = null;
        switch (vuMark) {
            case LEFT:
                cryptoToPit = new PathBuilder(stoneToCrypto.end())
                        .lineTo(new Vector2d(12 - CRYPTO_COL_WIDTH, -52 + 2 * CRYPTO_COL_WIDTH))
                        .lineTo(new Vector2d(12 - CRYPTO_COL_WIDTH, -12))
                        .turn(-Math.PI / 4)
                        .build();
                ultrasonicTarget = UltrasonicLocalizer.UltrasonicTarget.WALL;
                break;
            case CENTER:
                cryptoToPit = new PathBuilder(stoneToCrypto.end())
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -58 + Math.sqrt(3) * (CRYPTO_COL_WIDTH - 4)))
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -12))
                        .turn(Math.PI / 6)
                        .build();
                ultrasonicTarget = UltrasonicLocalizer.UltrasonicTarget.FULL_COLUMN;
                break;
            case RIGHT:
                cryptoToPit = new PathBuilder(stoneToCrypto.end())
                        .lineTo(new Vector2d(12, -52 + 2 * CRYPTO_COL_WIDTH))
                        .lineTo(new Vector2d(12 + CRYPTO_COL_WIDTH, -12))
                        .turn(Math.PI / 4)
                        .build();
                ultrasonicTarget = UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN;
                break;
        }

        Pose2d pitPose = cryptoToPit.end();
        Path pitToCrypto = new PathBuilder(cryptoToPit.end())
                .lineTo(new Vector2d(pitPose.x(), -56))
                .build();

        robot.drive.setEstimatedPose(stoneToCrypto.start());
        robot.drive.followPath(stoneToCrypto);
        robot.drive.waitForPathFollower();

        robot.dumpBed.dump();
        robot.sleep(0.5);

        robot.drive.followPath(cryptoToPit);
        robot.sleep(0.3 * cryptoToPit.duration());
        robot.dumpBed.retract();
        robot.intake.setIntakePower(1);
        robot.drive.waitForPathFollower();

        robot.drive.followPath(pitToCrypto);
        robot.sleep(0.2 * pitToCrypto.duration());
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.sleep(0.5 * pitToCrypto.duration());
        robot.drive.extendSideSwivel();
        ultrasonicLocalizer.setTarget(ultrasonicTarget);
        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.intake.setIntakePower(-1);
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
