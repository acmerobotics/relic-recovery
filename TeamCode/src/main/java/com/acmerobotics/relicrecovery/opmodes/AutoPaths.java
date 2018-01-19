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
    /** used to artificially adjust the balancing stone location */
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

    public static Path makeNormalPathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        int vuMarkInt = vuMarkMap.get(vuMark);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(cryptoboxX, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, -58))
                        .build();
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(-48, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-58, cryptoboxY))
                        .build();
            }
            case FAR_RED: {
                double cryptoboxY = 36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(-48, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(Math.PI / 2)
                        .lineTo(new Vector2d(-58, cryptoboxY))
                        .build();
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), 0))
                        .lineTo(new Vector2d(cryptoboxX, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, 58))
                        .build();
            }
        }
        return new Path(Collections.emptyList());
    }
}
