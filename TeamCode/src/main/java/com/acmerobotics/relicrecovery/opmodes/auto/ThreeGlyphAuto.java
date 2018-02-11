package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import static com.acmerobotics.relicrecovery.opmodes.AutoPaths.CRYPTO_COL_WIDTH;
import static com.acmerobotics.relicrecovery.opmodes.AutoPaths.VUMARK_MAP;

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
        RelicRecoveryVuMark vuMark = RelicRecoveryVuMark.CENTER;
        vuMark = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.RIGHT : vuMark;
        int vuMarkInt = VUMARK_MAP.get(vuMark);
        Pose2d stonePose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;

        Path stoneToCrypto;
        switch (vuMark) {
            case LEFT: {
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX - 18, -48))
                        .turn(-Math.PI / 4)
                        .lineTo(new Vector2d(cryptoboxX - 8, -58))
                        .build();
                break;
            }
            case CENTER:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX + 4 + 10 / Math.sqrt(3), -48))
                        .turn(-2 * Math.PI / 3)
                        .lineTo(new Vector2d(cryptoboxX + 4, -58))
                        .build();
                break;
            case RIGHT:
                stoneToCrypto = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX + 18, -48))
                        .turn(-3 * Math.PI / 4)
                        .lineTo(new Vector2d(cryptoboxX + 8, -58))
                        .build();
                break;
        }

        Path cryptoToPit;
        switch (vuMark) {
            case LEFT: {
                cryptoToPit = new PathBuilder(new Pose2d(cryptoboxX + 18, -58, -Math.PI / 4))
                        .lineTo(new Vector2d(cryptoboxX - 18, -48))
                        .turn(-Math.PI / 4)
                        .lineTo(new Vector2d(cryptoboxX - 8, -58))
                        .build();
                break;
            }
            case CENTER:
                cryptoToPit = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX + 4 + 10 / Math.sqrt(3), -48))
                        .turn(-2 * Math.PI / 3)
                        .lineTo(new Vector2d(cryptoboxX + 4, -58))
                        .build();
                break;
            case RIGHT:
                cryptoToPit = new PathBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX + 18, -48))
                        .turn(-3 * Math.PI / 4)
                        .lineTo(new Vector2d(cryptoboxX + 8, -58))
                        .build();
                break;
        }
    }
}
