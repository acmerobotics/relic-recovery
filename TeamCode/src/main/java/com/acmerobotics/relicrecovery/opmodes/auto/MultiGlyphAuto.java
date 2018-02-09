package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
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
                .lineTo(new Vector2d(12, -44))
                .build();

        robot.drive.followPath(pitToCrypto1);

        sleep((int) (1000 * 0.5 * pitToCrypto1.duration()));

        robot.intake.setIntakePower(-1);

        waitForPathFollower();

        robot.intake.setIntakePower(0);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);

        sleep(500);

        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.WALL);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        followPathSync(new PathBuilder(new Pose2d(12, -44, Math.PI / 2))
                .lineTo(new Vector2d(12, -57))
                .build());

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        alignWithColumnSync();

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

        waitForPathFollower();

        robot.intake.setIntakePower(0);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);

        sleep(500);

        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        followPathSync(new PathBuilder(new Pose2d(12, -36, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -36))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -57))
                .build());

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        alignWithColumnSync();

        robot.dumpBed.dump();
        sleep(1000);

        followPathSync(new PathBuilder(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -57, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44))
                .build());

        robot.dumpBed.retract();
    }
}
