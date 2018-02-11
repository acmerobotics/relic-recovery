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
    /** used to artificially adjust the balancing stone location. */
    public static double STONE_CORRECTION = 0.5; // in

    public static double CRYPTO_COL_WIDTH = 7.5; // in

    public static Vector2d ORTHOGONAL_SCORING_OFFSET = new Vector2d(16, 0);
    public static Vector2d DIAGONAL_SCORING_OFFSET = new Vector2d(14, 8);

    public static final Map<RelicRecoveryVuMark, Integer> VUMARK_MAP;

    static {
        VUMARK_MAP = new HashMap<>();
        VUMARK_MAP.put(RelicRecoveryVuMark.LEFT, 1);
        VUMARK_MAP.put(RelicRecoveryVuMark.CENTER, 0);
        VUMARK_MAP.put(RelicRecoveryVuMark.RIGHT, -1);
    }

    public static Pose2d getBalancingStonePose(BalancingStone stone) {
        if (stone == BalancingStone.NEAR_BLUE || stone == BalancingStone.FAR_BLUE) {
            return new Pose2d(stone.getPosition(), Math.PI);
        } else {
            return new Pose2d(stone.getPosition(), 0);
        }
    }

    public static Pose2d getAdjustedBalancingStonePose(BalancingStone stone) {
        return getBalancingStonePose(stone).added(new Pose2d(STONE_CORRECTION, 0, 0));
    }

    public static Path makeNormalPathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        vuMark = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        int vuMarkInt = VUMARK_MAP.get(vuMark);
        Pose2d stonePose = getAdjustedBalancingStonePose(stone);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, -57))
                        .build();
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(stonePose)
                        .lineTo(new Vector2d(-48, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-57, cryptoboxY))
                        .build();
            }
            case FAR_RED: {
                double cryptoboxY = 36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(stonePose)
                        .lineTo(new Vector2d(-48, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(Math.PI / 2)
                        .lineTo(new Vector2d(-57, cryptoboxY))
                        .build();
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new PathBuilder(stonePose)
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
        int vuMarkInt = VUMARK_MAP.get(vuMark);
        Pose2d stonePose = getAdjustedBalancingStonePose(stone);
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
                break;
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 14))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 8))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 14))
                            .turn(3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 8))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 14))
                            .turn(3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 8))
                            .build();
                }
                break;
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 18, 48))
                            .turn(-Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX + 8, 58))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 4 + 10 / Math.sqrt(3), 48))
                            .turn(-Math.PI / 3)
                            .lineTo(new Vector2d(cryptoboxX + 4, 58))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX - 18, 48))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX - 8, 58))
                            .build();
                }
                break;
            }
            case FAR_RED: {
                double cryptoboxY = 36 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, 48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 14))
                            .turn(Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 8))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, 48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 14))
                            .turn(Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 8))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new PathBuilder(stonePose)
                            .lineTo(new Vector2d(-52, 48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 14))
                            .turn(-Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 8))
                            .build();
                }
                break;
            }
        }
        return new Path(Collections.emptyList());
    }
}
