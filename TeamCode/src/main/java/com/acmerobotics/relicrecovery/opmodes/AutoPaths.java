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
    public static double STONE_CORRECTION = 0; // in

    public static double CRYPTO_COL_WIDTH = 7.5; // in

    public static final Map<RelicRecoveryVuMark, Integer> vuMarkMap;

    static {
        vuMarkMap = new HashMap<>();
        vuMarkMap.put(RelicRecoveryVuMark.LEFT, 1);
        vuMarkMap.put(RelicRecoveryVuMark.CENTER, 0);
        vuMarkMap.put(RelicRecoveryVuMark.RIGHT, -1);
    }

    public static Path makeNormalPathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        vuMark = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        int vuMarkInt = vuMarkMap.get(vuMark);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(cryptoboxX, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, -57))
                        .build();
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(-48, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-57, cryptoboxY))
                        .build();
            }
            case FAR_RED: {
                double cryptoboxY = 36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(-48, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(Math.PI / 2)
                        .lineTo(new Vector2d(-57, cryptoboxY))
                        .build();
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), 0))
                        .lineTo(new Vector2d(cryptoboxX, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, 57))
                        .build();
            }
        }
        return new Path(Collections.emptyList());
    }

    public static Path makeDiagonalPathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        vuMark = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.RIGHT : vuMark;
        int vuMarkInt = vuMarkMap.get(vuMark);
        Pose2d stonePose = new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX - 18, -48))
                            .turn(-Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX - 8, -58))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 4 + 10 / Math.sqrt(3), -48))
                            .turn(-2 * Math.PI / 3)
                            .lineTo(new Vector2d(cryptoboxX + 4, -58))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 18, -48))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX + 8, -58))
                            .build();
                }
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 12))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 6))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 9))
                            .turn(3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 3))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 12))
                            .turn(3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 6))
                            .build();
                }
                return new PathBuilder(new Pose2d(stone.getPosition().added(new Vector2d(STONE_CORRECTION, 0)), Math.PI))
                        .lineTo(new Vector2d(-48 - CRYPTO_COL_WIDTH * vuMarkInt, -48))
                        .turn(3 * Math.PI / 4)
                        .lineTo(new Vector2d(-60, cryptoboxY))
                        .build();
            }
            // TODO: add red once path endpoints are determined
        }
        throw new UnsupportedOperationException("implement red diagonal paths");
//        return new Path(Collections.emptyList());
    }
}
