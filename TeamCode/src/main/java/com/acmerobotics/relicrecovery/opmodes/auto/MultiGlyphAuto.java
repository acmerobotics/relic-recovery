package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class MultiGlyphAuto extends AutoOpMode {
    @Override
    protected void setup() {
        robot.drive.setEstimatedPose(new Pose2d(48 + AutoPaths.STONE_CORRECTION, -48, Math.PI));
    }

    @Override
    protected void run() {
        followPathSync(new PathBuilder(new Pose2d(48, -48, Math.PI))
                .lineTo(new Vector2d(12, -48))
                .turn(-Math.PI / 2)
                .lineTo(new Vector2d(12, -24))
                .lineTo(new Vector2d(12, -56))
                .build());

        sleep(1500);

        followPathSync(new PathBuilder(new Pose2d(12, -56, Math.PI / 2))
                .lineTo(new Vector2d(12, -12))
                .lineTo(new Vector2d(12, -24))
                .lineTo(new Vector2d(12, 0))
                .lineTo(new Vector2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -56))
                .build());

//        sleep(1500);
//
//        followPathSync(new PathBuilder(new Pose2d(12 + AutoPaths.CRYPTO_COL_WIDTH, -56, Math.PI / 2))
//                .lineTo(new Vector2d())));
    }
}
