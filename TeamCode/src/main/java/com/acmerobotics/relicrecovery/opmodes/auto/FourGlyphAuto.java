package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.RelicRecoverer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Disabled
@Autonomous(name = "4 Glyph Auto")
public class FourGlyphAuto extends AutoOpMode {
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

        robot.drive.followTrajectory(new TrajectoryBuilder(initialPose)
                .lineTo(new Vector2d(12, -48))
                .build());
        robot.drive.waitForPathFollower();

        robot.intake.setIntakePower(1);

        robot.drive.followTrajectory(new TrajectoryBuilder(new Pose2d(12, -48, Math.PI))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(12, -12))
                .turn(-Math.PI / 4)
                .build());
        robot.drive.waitForPathFollower();
//        followPathSync(new PathBuilder(new Pose2d(12, -48, Math.PI / 2))
//                .lineTo(new Vector2d(12, -12))
//                .build());

        Trajectory pitToCrypto1 = new TrajectoryBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12, -36))
                .build();

        robot.drive.followTrajectory(pitToCrypto1);

        sleep((int) (1000 * 0.5 * pitToCrypto1.duration()));

        robot.intake.setIntakePower(-1);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.drive.extendSideSwivel();

        robot.drive.waitForPathFollower();

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.WALL);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

//        Pose2d estimatedPose = robot.drive.getEstimatedPose();
        robot.drive.followTrajectory(new TrajectoryBuilder(new Pose2d(12, -36, Math.PI / 2))
                .lineTo(new Vector2d(12, -56))
                .build());
        robot.drive.waitForPathFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        robot.drive.enableHeadingCorrection(Math.PI / 2);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(12, -56));

        robot.drive.retractSideSwivel();
        robot.dumpBed.dump();
        sleep(500);

        Trajectory cryptoToPit1 = new TrajectoryBuilder(new Pose2d(12, -56, Math.PI / 2))
                .lineTo(new Vector2d(12, -12))
                .build();
        robot.drive.followTrajectory(cryptoToPit1);

        sleep((int) (1000 * 0.25 * cryptoToPit1.duration()));

        robot.dumpBed.retract();
        robot.intake.setIntakePower(1);

        robot.drive.waitForPathFollower();

        robot.drive.followTrajectory(new TrajectoryBuilder(new Pose2d(12, -12, Math.PI / 2))
                .turn(-Math.PI / 4)
                .forward(18)
                .back(18)
                .turn(Math.PI / 4)
                .build());
        robot.drive.waitForPathFollower();

        Trajectory pitToCrypto2 = new TrajectoryBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12, -36))
                .build();

        robot.drive.followTrajectory(pitToCrypto2);

        sleep((int) (1000 * 0.5 * pitToCrypto2.duration()));

        robot.intake.setIntakePower(-1);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.drive.extendSideSwivel();

        robot.drive.waitForPathFollower();

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.setTarget(UltrasonicLocalizer.UltrasonicTarget.EMPTY_COLUMN);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        robot.waitOneFullCycle();

//        estimatedPose = robot.drive.getEstimatedPose();
        robot.drive.followTrajectory(new TrajectoryBuilder(new Pose2d(12, -36, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -36))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -56))
                .build());
        robot.drive.waitForPathFollower();

        ultrasonicLocalizer.disableUltrasonicFeedback();
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.UP);
        robot.drive.enableHeadingCorrection(Math.PI / 2);

        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

        robot.drive.disableHeadingCorrection();
        robot.drive.setEstimatedPosition(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -56));

        robot.drive.retractSideSwivel();
        robot.dumpBed.dump();
        sleep(500);

        robot.drive.followTrajectory(new TrajectoryBuilder(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -56, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -48))
                .build());
        robot.drive.waitForPathFollower();

        robot.dumpBed.retract();
        robot.drive.retractSideSwivel();

        telemetry.log().add(String.format("Took %.2fs", TimestampedData.getCurrentTime() - startTime));
        telemetry.update();

        while (opModeIsActive());
    }
}
