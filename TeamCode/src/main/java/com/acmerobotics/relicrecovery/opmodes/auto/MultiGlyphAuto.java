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
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
@Autonomous
public class MultiGlyphAuto extends AutoOpMode {
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

        robot.relicRecoverer.setWristPosition(RelicRecoverer.WristPosition.UP);

        Pose2d initialPose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        robot.drive.setEstimatedPose(initialPose);

        followPathSync(new PathBuilder(initialPose)
                .lineTo(new Vector2d(12, -48))
                .build());

        robot.intake.setIntakePower(1);

        followPathSync(new PathBuilder(new Pose2d(12, -48, Math.PI))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(12, -12))
                .turn(-Math.PI / 4)
                .build());

        Path pitToCrypto1 = new PathBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12, -36))
                .build();

        robot.drive.followPath(pitToCrypto1);

        sleep((int) (1000 * 0.5 * pitToCrypto1.duration()));

        robot.intake.setIntakePower(-1);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.drive.extendSideSwivel();

        waitForPathFollower();

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.WALL);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        Pose2d estimatedPose = robot.drive.getEstimatedPose();
        followPathSync(new PathBuilder(new Pose2d(estimatedPose.x(), estimatedPose.y(), Math.PI / 2))
                .lineTo(new Vector2d(12, -57))
                .build());

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        alignWithColumnSync();

        robot.drive.setEstimatedPosition(new Vector2d(12, -57));

        robot.drive.retractSideSwivel();
        robot.dumpBed.dump();
        sleep(500);

        Path cryptoToPit1 = new PathBuilder(new Pose2d(12, -57, Math.PI / 2))
                .lineTo(new Vector2d(12, -12))
                .build();
        robot.drive.followPath(cryptoToPit1);

        sleep((int) (1000 * 0.5 * cryptoToPit1.duration()));

        robot.dumpBed.retract();
        robot.intake.setIntakePower(1);

        waitForPathFollower();

        followPathSync(new PathBuilder(new Pose2d(12, -12, Math.PI / 2))
                .turn(-Math.PI / 4)
                .forward(18)
                .back(18)
                .turn(Math.PI / 4)
                .build());

        Path pitToCrypto2 = new PathBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12, -36))
                .build();

        robot.drive.followPath(pitToCrypto2);

        sleep((int) (1000 * 0.5 * pitToCrypto2.duration()));

        robot.intake.setIntakePower(-1);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.drive.extendSideSwivel();

        waitForPathFollower();

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

        estimatedPose = robot.drive.getEstimatedPose();
        followPathSync(new PathBuilder(new Pose2d(estimatedPose.x(), estimatedPose.y(), Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, estimatedPose.y()))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -57))
                .build());

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        alignWithColumnSync();

        robot.drive.setEstimatedPosition(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -57));

        robot.drive.retractSideSwivel();
        robot.dumpBed.dump();
        sleep(500);

        followPathSync(new PathBuilder(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -57, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -48))
                .build());

        robot.dumpBed.retract();
        robot.drive.retractSideSwivel();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
