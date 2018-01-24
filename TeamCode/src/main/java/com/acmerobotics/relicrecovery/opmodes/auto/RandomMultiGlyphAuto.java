package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class RandomMultiGlyphAuto extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        robot.drive.setEstimatedPose(new Pose2d(48 + AutoPaths.STONE_CORRECTION, -48, Math.PI));

        followPathSync(new PathBuilder(new Pose2d(48, -48, Math.PI))
                .lineTo(new Vector2d(12, -48))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(12, -12))
                .build());

        while (opModeIsActive()) {
            int choice = (int) (3 * Math.random());
            choice--;
            Vector2d v = new Vector2d(12 + choice * CryptoboxTracker.ACTUAL_RAIL_GAP, -56);

            sleep(1500);

            followPathSync(new PathBuilder(new Pose2d(12, -12, Math.PI / 2)).lineTo(v).build());

            sleep(1500);

            followPathSync(new PathBuilder(new Pose2d(v, Math.PI / 2)).lineTo(new Vector2d(12, -12)).build());
        }
    }
}
