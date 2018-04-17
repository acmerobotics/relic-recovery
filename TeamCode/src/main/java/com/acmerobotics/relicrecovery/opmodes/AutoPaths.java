package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

@Config
public class AutoPaths {
    /** used to artificially adjust the balancing stone location. */
    public static double STONE_CORRECTION = -3; // in

    public static double CRYPTO_COL_WIDTH = 7.5; // in

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

    public static Vector2d getCryptoboxColumnPosition(Cryptobox cryptobox, RelicRecoveryVuMark column) {
        if (column == RelicRecoveryVuMark.UNKNOWN) {
            throw new IllegalArgumentException("Column may not be UNKNOWN");
        }
        int columnInt = VUMARK_MAP.get(column);
        switch (cryptobox) {
            case NEAR_BLUE:
                return cryptobox.getPosition().added(new Vector2d(columnInt * CRYPTO_COL_WIDTH, 0));
            case NEAR_RED:
                return cryptobox.getPosition().added(new Vector2d(-columnInt * CRYPTO_COL_WIDTH, 0));
            case FAR_BLUE:
            case FAR_RED:
                return cryptobox.getPosition().added(new Vector2d(0, -columnInt * CRYPTO_COL_WIDTH));
        }
        throw new UnsupportedOperationException(cryptobox + " is not a supported cryptobox");
    }

    public static Pose2d getAdjustedBalancingStonePose(BalancingStone stone) {
        return getBalancingStonePose(stone).added(new Pose2d(STONE_CORRECTION, 0, 0));
    }

    public static Trajectory makeNormalTrajectoryToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        vuMark = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.CENTER : vuMark;
        int vuMarkInt = VUMARK_MAP.get(vuMark);
        Pose2d stonePose = getAdjustedBalancingStonePose(stone);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;
                return new TrajectoryBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, -57))
                        .build();
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new TrajectoryBuilder(stonePose)
                        .lineTo(new Vector2d(-48, -48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-57, cryptoboxY))
                        .build();
            }
            case FAR_RED: {
                double cryptoboxY = 36 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new TrajectoryBuilder(stonePose)
                        .lineTo(new Vector2d(-48, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(-48, cryptoboxY))
                        .turn(Math.PI / 2)
                        .lineTo(new Vector2d(-57, cryptoboxY))
                        .build();
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - CRYPTO_COL_WIDTH * vuMarkInt;
                return new TrajectoryBuilder(stonePose)
                        .lineTo(new Vector2d(cryptoboxX, 48))
                        .turn(-Math.PI / 2)
                        .lineTo(new Vector2d(cryptoboxX, 57))
                        .build();
            }
        }
        return new Trajectory(Collections.emptyList());
    }

    public static Trajectory makeDiagonalTrajectoryToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        vuMark = vuMark == RelicRecoveryVuMark.UNKNOWN ? RelicRecoveryVuMark.RIGHT : vuMark;
        int vuMarkInt = VUMARK_MAP.get(vuMark);
        Pose2d stonePose = getAdjustedBalancingStonePose(stone);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 + CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX - 19, -48))
                            .turn(-Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX - 9, -58))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 4 + 8 / Math.sqrt(3), -48))
                            .turn(-2 * Math.PI / 3)
                            .lineTo(new Vector2d(cryptoboxX + 4, -56))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 21, -48))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX + 10, -59))
                            .build();
                }
                break;
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 13 + AutoOpMode.LATERAL_BIAS))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 7 + AutoOpMode.LATERAL_BIAS))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 14 + AutoOpMode.LATERAL_BIAS))
                            .turn(3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 8 + AutoOpMode.LATERAL_BIAS))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(-52, -48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 13 + AutoOpMode.LATERAL_BIAS))
                            .turn(3 * Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 7 + AutoOpMode.LATERAL_BIAS))
                            .build();
                }
                break;
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 16, 48))
                            .turn(-Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX + 6, 58))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX + 4 + 8 / Math.sqrt(3), 48))
                            .turn(-Math.PI / 3)
                            .lineTo(new Vector2d(cryptoboxX + 4, 56))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(cryptoboxX - 21, 48))
                            .turn(-3 * Math.PI / 4)
                            .lineTo(new Vector2d(cryptoboxX - 10, 59))
                            .build();
                }
                break;
            }
            case FAR_RED: {
                double cryptoboxY = 36 - CRYPTO_COL_WIDTH * vuMarkInt;
                if (vuMark == RelicRecoveryVuMark.LEFT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(-52, 48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 13 - AutoOpMode.LATERAL_BIAS))
                            .turn(Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 7 - AutoOpMode.LATERAL_BIAS))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.CENTER) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(-52, 48))
                            .lineTo(new Vector2d(-52, cryptoboxY + 14 - AutoOpMode.LATERAL_BIAS))
                            .turn(Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY + 8 - AutoOpMode.LATERAL_BIAS))
                            .build();
                } else if (vuMark == RelicRecoveryVuMark.RIGHT) {
                    return new TrajectoryBuilder(stonePose)
                            .lineTo(new Vector2d(-52, 48))
                            .lineTo(new Vector2d(-52, cryptoboxY - 13 - AutoOpMode.LATERAL_BIAS))
                            .turn(-Math.PI / 4)
                            .lineTo(new Vector2d(-58, cryptoboxY - 7 - AutoOpMode.LATERAL_BIAS))
                            .build();
                }
                break;
            }
        }
        return new Trajectory(Collections.emptyList());
    }
}
