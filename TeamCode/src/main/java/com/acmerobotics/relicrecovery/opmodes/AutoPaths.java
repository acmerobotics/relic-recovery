package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.configuration.BalancingStone;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path.LineSegment;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.acmerobotics.relicrecovery.path.WaitSegment;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Arrays;
import java.util.Collections;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Ryan
 */

public class AutoPaths {
    public static Map<RelicRecoveryVuMark, Integer> vuMarkMap;

    static {
        vuMarkMap = new HashMap<>();
        vuMarkMap.put(RelicRecoveryVuMark.LEFT, -1);
        vuMarkMap.put(RelicRecoveryVuMark.CENTER, 0);
        vuMarkMap.put(RelicRecoveryVuMark.RIGHT, 1);
        vuMarkMap.put(RelicRecoveryVuMark.UNKNOWN, 0);
    }

    public static Path makePathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        int vuMarkInt = vuMarkMap.get(vuMark);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 - 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(48, -48, Math.PI), new Pose2d(cryptoboxX, -48, Math.PI)),
                        new PointTurn(new Pose2d(cryptoboxX, -48, Math.PI), -Math.PI / 2), // + Math.toRadians(10)),
                        new WaitSegment(new Pose2d(cryptoboxX, -48, Math.PI / 2), 5),
                        new LineSegment(new Pose2d(cryptoboxX, -48, Math.PI / 2), new Pose2d(cryptoboxX, -62, Math.PI / 2), true)
                ));
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 + 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(-24, -48, Math.PI), new Pose2d(-50, -48, Math.PI)),
                        new PointTurn(new Pose2d(-50, -48, Math.PI), -Math.PI / 2),
                        new LineSegment(new Pose2d(-50, -48, Math.PI / 2), new Pose2d(-50, cryptoboxY, Math.PI / 2)),
                        new PointTurn(new Pose2d(-50, cryptoboxY, -Math.PI / 2), -Math.PI / 2),
                        new LineSegment(new Pose2d(-50, cryptoboxY, 0), new Pose2d(-62, cryptoboxY, 0), true)
                ));
            }
            case FAR_RED: {
                double cryptoboxY = 36 + 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(-24, 48, 0), new Pose2d(-50, -48, 0), true),
                        new PointTurn(new Pose2d(-50, 48, 0), -Math.PI / 2),
                        new LineSegment(new Pose2d(-50, 48, -Math.PI / 2), new Pose2d(-50, cryptoboxY, -Math.PI / 2)),
                        new PointTurn(new Pose2d(-50, cryptoboxY, Math.PI / 2), Math.PI / 2),
                        new LineSegment(new Pose2d(-50, cryptoboxY, 0), new Pose2d(-62, cryptoboxY, 0), true)
                ));
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(48, 48, 0), new Pose2d(cryptoboxX, 48, 0), true),
                        new PointTurn(new Pose2d(cryptoboxX, 48, 0), -Math.PI / 2),
                        new WaitSegment(new Pose2d(cryptoboxX, 48, -Math.PI / 2), 5),
                        new LineSegment(new Pose2d(cryptoboxX, 48, -Math.PI / 2), new Pose2d(cryptoboxX, 62, -Math.PI / 2), true)
                ));
            }
        }
        return new Path(Collections.emptyList());
    }

    public static Path makeCryptoboxRetreat(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        int vuMarkInt = vuMarkMap.get(vuMark);
        switch (stone) {
            case NEAR_BLUE: {
                double cryptoboxX = 12 - 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(cryptoboxX, -60, Math.PI / 2), new Pose2d(cryptoboxX, -54, Math.PI / 2))
                ));
            }
            case FAR_BLUE: {
                double cryptoboxY = -36 + 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(-60, cryptoboxY, 0), new Pose2d(-54, cryptoboxY, 0))
                ));
            }
            case FAR_RED: {
                double cryptoboxY = 36 + 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(-60, cryptoboxY, 0), new Pose2d(-54, cryptoboxY, 0))
                ));
            }
            case NEAR_RED: {
                double cryptoboxX = 12 - 7.5 * vuMarkInt;
                return new Path(Arrays.asList(
                        new LineSegment(new Pose2d(cryptoboxX, 60, -Math.PI / 2), new Pose2d(cryptoboxX, 54, -Math.PI / 2))
                ));
            }
        }
        return new Path(Collections.emptyList());
    }

    private static Path makePathToCryptoboxOld(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        switch (stone) {
            case NEAR_BLUE:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(48, -48, Math.PI),
                        new Pose2d(12 - 7.5 * vuMarkMap.get(vuMark), -48),
                        new Pose2d(12 - 7.5 * vuMarkMap.get(vuMark), -60, -Math.PI / 2)
                ));
            case FAR_BLUE:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(-24, -48, Math.PI),
                        new Pose2d(-50, -48),
                        new Pose2d(-50, -36 + 7.5 * vuMarkMap.get(vuMark)),
                        new Pose2d(-60, -36 + 7.5 * vuMarkMap.get(vuMark), Math.PI)
                ));
            case FAR_RED:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(-24, 48, 0),
                        new Pose2d(-50, 48),
                        new Pose2d(-50, 36 + 7.5 * vuMarkMap.get(vuMark)),
                        new Pose2d(-60, 36 + 7.5 * vuMarkMap.get(vuMark), Math.PI)
                ));
            case NEAR_RED:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(48, 48, 0),
                        new Pose2d(12 - 7.5 * vuMarkMap.get(vuMark), 48),
                        new Pose2d(12 - 7.5 * vuMarkMap.get(vuMark), 60, Math.PI / 2)
                ));
        }
        return null;
    }
}
