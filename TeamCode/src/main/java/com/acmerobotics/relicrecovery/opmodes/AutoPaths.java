package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

@Config
public class AutoPaths {
    public static double STONE_CORRECTION = 3; // in
    public static double CRYPTO_COL_WIDTH = 7.5; // in

    public static final Map<RelicRecoveryVuMark, Integer> vuMarkMap;

    static {
        vuMarkMap = new HashMap<>();
        vuMarkMap.put(RelicRecoveryVuMark.LEFT, 1);
        vuMarkMap.put(RelicRecoveryVuMark.CENTER, 0);
        vuMarkMap.put(RelicRecoveryVuMark.RIGHT, -1);
        vuMarkMap.put(RelicRecoveryVuMark.UNKNOWN, 0);
    }

    public static Path makePathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        int vuMarkInt = vuMarkMap.get(vuMark);
        switch (stone) {
            case NEAR_BLUE: {
                return new PathBuilder(new Pose2d(48, -48, Math.PI))
                        .lineTo(new Vector2d(12 + STONE_CORRECTION + CRYPTO_COL_WIDTH * vuMarkInt, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(12 + STONE_CORRECTION + CRYPTO_COL_WIDTH * vuMarkInt, -58))
                        .build();
            }
            case FAR_BLUE: {
                return new PathBuilder(new Pose2d(-24, -48, Math.PI))
                        .lineTo(new Vector2d(-48 + STONE_CORRECTION, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48 + STONE_CORRECTION, -36 - CRYPTO_COL_WIDTH * vuMarkInt))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-58 + STONE_CORRECTION, -36 - CRYPTO_COL_WIDTH  * vuMarkInt))
                        .build();
            }
            case FAR_RED: {
                return new PathBuilder(new Pose2d(-24, 48, Math.PI))
                        .lineTo(new Vector2d(-48 + STONE_CORRECTION, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48 + STONE_CORRECTION, 36 - CRYPTO_COL_WIDTH * vuMarkInt))
                        .turn(Math.PI / 2)
                        .lineTo(new Vector2d(-58 + STONE_CORRECTION, 36 - CRYPTO_COL_WIDTH  * vuMarkInt))
                        .build();
            }
            case NEAR_RED: {
                return new PathBuilder(new Pose2d(48, 48, 0))
                        .lineTo(new Vector2d(12 + STONE_CORRECTION - CRYPTO_COL_WIDTH * vuMarkInt, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(12 + STONE_CORRECTION - CRYPTO_COL_WIDTH * vuMarkInt, 58))
                        .build();
            }
        }
        return new Path(Collections.emptyList());
    }
}
