package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.vision.CryptoboxLocalizer;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Config
@Autonomous
public class MultiGlyphAuto extends AutoOpMode {
    public static boolean VISION = false;

    private CryptoboxLocalizer cryptoLocalizer;
    private UltrasonicLocalizer ultrasonicLocalizer;
    private CryptoboxTracker cryptoTracker;

    @Override
    protected void setup() {
        ultrasonicLocalizer = new UltrasonicLocalizer(robot.drive, MecanumDrive.DISTANCE_SMOOTHER_COEFF);
        robot.drive.setLocalizer(ultrasonicLocalizer);
        robot.drive.setEstimatedPosition(BalancingStone.NEAR_BLUE.getPosition());

        if (VISION) {
            cryptoTracker = new CryptoboxTracker(AllianceColor.BLUE);
            cryptoTracker.disable();
            cryptoLocalizer = new CryptoboxLocalizer(cryptoTracker, camera.getProperties(), robot.drive);
            cryptoLocalizer.addListener((pos, timestamp) -> {
                if (pos != null && Vector2d.distance(pos, robot.drive.getEstimatedPosition()) < 16) {
                    robot.drive.setEstimatedPosition(pos);
                }
            });
            camera.addTracker(cryptoTracker);
        }
    }

    @Override
    protected void run() {
        Pose2d initialPose = AutoPaths.getAdjustedBalancingStonePose(BalancingStone.NEAR_BLUE);
        robot.drive.setEstimatedPose(initialPose);

        followPathSync(new PathBuilder(initialPose)
                .lineTo(new Vector2d(12, -48))
                .build());

        robot.intake.setIntakePower(1);

        followPathSync(new PathBuilder(new Pose2d(12, -48, Math.PI))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(12, -12))
                .build());

        robot.intake.setIntakePower(-1);

        followPathSync(new PathBuilder(new Pose2d(12, -12, 3 * Math.PI / 4))
                .turn(-Math.PI / 4)
                .lineTo(new Vector2d(12, -44))
                .build());

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        if (VISION) {
            cryptoTracker.enable();
            sleep(1000);
            cryptoTracker.disable();
        }

        followPathSync(new PathBuilder(new Pose2d(12, -44, Math.PI / 2))
                .lineTo(new Vector2d(12, -54))
                .build());

        ultrasonicLocalizer.disableUltrasonicFeedback();
        alignWithColumnSync();

//        robot.dumpBed.dump();
        sleep(1000);

        followPathSync(new PathBuilder(new Pose2d(12, -54, Math.PI / 2))
                .lineTo(new Vector2d(12, -44))
                .build());

//        robot.dumpBed.retract();

        robot.intake.setIntakePower(1);

        followPathSync(new PathBuilder(new Pose2d(12, -44, Math.PI / 2))
                .lineTo(new Vector2d(12, -12))
                .turn(-Math.PI / 4)
                .forward(9)
                .back(9)
                .turn(Math.PI / 4)
                .build());

        robot.intake.setIntakePower(-1);

        followPathSync(new PathBuilder(new Pose2d(12, -12, Math.PI / 2))
                .lineTo(new Vector2d(12, -36))
                .build());

        robot.intake.setIntakePower(0);
        ultrasonicLocalizer.enableUltrasonicFeedback();

        if (VISION) {
            robot.phoneSwivel.pointAtCryptobox();
            sleep(500);
            cryptoTracker.enable();
            sleep(1000);
            cryptoTracker.disable();
            robot.phoneSwivel.pointAtJewel();
            sleep(500);
        }

        followPathSync(new PathBuilder(new Pose2d(12, -36, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -36))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -54))
                .build());

        ultrasonicLocalizer.disableUltrasonicFeedback();
        alignWithColumnSync();

//        robot.dumpBed.dump();
        sleep(1000);

        followPathSync(new PathBuilder(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -54, Math.PI / 2))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -44))
                .build());

//        robot.dumpBed.retract();
    }
}
