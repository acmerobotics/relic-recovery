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
import com.acmerobotics.relicrecovery.subsystems.Intake;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "4 Glyph Auto (Optimized)")
public class OptimizedFourGlyphAuto extends AutoOpMode {
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

        Pose2d initialPose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        robot.drive.setEstimatedPose(initialPose);

        robot.drive.followPath(new PathBuilder(initialPose)
                .lineTo(new Vector2d(12, initialPose.y()))
                .build());
        robot.drive.waitForPathFollower();

        robot.intake.autoIntake();

        robot.drive.followPath(new PathBuilder(new Pose2d(12, initialPose.y(), Math.PI))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(12, -12))
                .turn(-Math.PI / 4)
                .build());
        robot.drive.waitForPathFollower();

        Path pitToCrypto1 = new PathBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12, -54))
                .build();

        robot.drive.followPath(pitToCrypto1);

        robot.drive.extendUltrasonicSwivel();
        robot.sleep(0.4 * pitToCrypto1.duration());

        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.intake.setIntakePower(-0.3);
        robot.sleep(0.5);
        robot.intake.setIntakePower(1);
        robot.drive.extendProximitySwivel();
        robot.drive.waitForPathFollower();
        robot.intake.setIntakePower(0);

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.drive.retractUltrasonicSwivel();
        robot.drive.enableHeadingCorrection(Math.PI / 2);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(12, -54));

        robot.drive.retractProximitySwivel();
        robot.dumpBed.dump();
        sleep(500);

        Path cryptoToPit1 = new PathBuilder(new Pose2d(12, -54, Math.PI / 2))
                .lineTo(new Vector2d(12, -12))
                .turn(-Math.PI / 4)
                .build();
        robot.drive.followPath(cryptoToPit1);

        robot.sleep(0.2 * cryptoToPit1.duration());

        robot.dumpBed.retract();
        robot.intake.autoIntake();

        robot.drive.waitForPathFollower();

        if (robot.intake.getMode() == Intake.Mode.AUTO) {
            // we still didn't get enough glyphs; let's forage a little more
            robot.drive.followPath(new PathBuilder(new Pose2d(12, -12, Math.PI / 4))
                    .forward(18)
                    .back(18)
                    .turn(Math.PI / 4)
                    .build());
        } else {
            robot.drive.followPath(new PathBuilder(new Pose2d(12, -12, Math.PI / 4))
                    .turn(Math.PI / 4)
                    .build());
        }
        robot.drive.waitForPathFollower();

        Path pitToCrypto2 = new PathBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -36))
                .build();

        robot.drive.followPath(pitToCrypto2);
        robot.drive.extendUltrasonicSwivel();

        robot.sleep(0.5 * pitToCrypto2.duration());

        robot.intake.setIntakePower(-1);
        robot.drive.extendProximitySwivel();

        robot.drive.waitForPathFollower();

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        Pose2d estimatedPose = robot.drive.getEstimatedPose();
        robot.drive.followPath(new PathBuilder(new Pose2d(estimatedPose.pos(), Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -54))
                .build());
        robot.drive.waitForPathFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.drive.retractUltrasonicSwivel();
        robot.drive.enableHeadingCorrection(Math.PI / 2);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -54));

        robot.dumpBed.dump();
        sleep(500);

        robot.drive.followPath(new PathBuilder(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -54, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -48))
                .build());
        robot.drive.waitForPathFollower();

        robot.dumpBed.retract();
        robot.drive.retractProximitySwivel();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
