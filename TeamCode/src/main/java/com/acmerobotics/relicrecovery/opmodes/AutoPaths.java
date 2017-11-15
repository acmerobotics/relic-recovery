package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.configuration.BalancingStone;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path.Path;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

import java.util.Arrays;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Ryan
 */

public class AutoPaths {
    public static Map<RelicRecoveryVuMark, Integer> vuMarkMap;

    static {
        vuMarkMap = new HashMap<>();
        vuMarkMap.put(RelicRecoveryVuMark.LEFT, 1);
        vuMarkMap.put(RelicRecoveryVuMark.CENTER, 0);
        vuMarkMap.put(RelicRecoveryVuMark.RIGHT, -1);
    }

    public static Path makePathToCryptobox(BalancingStone stone, RelicRecoveryVuMark vuMark) {
        switch (stone) {
            case NEAR_BLUE:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(48, -48, Math.PI),
                        new Pose2d(12 + 7.5 * vuMarkMap.get(vuMark), -48),
                        new Pose2d(12 + 7.5 * vuMarkMap.get(vuMark), -60, -Math.PI / 2)
                ));
            case FAR_BLUE:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(-24, -48, Math.PI),
                        new Pose2d(-50, -48),
                        new Pose2d(-50, -36 - 7.5 * vuMarkMap.get(vuMark)),
                        new Pose2d(-60, -36 - 7.5 * vuMarkMap.get(vuMark), Math.PI)
                ));
            case FAR_RED:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(-24, 48, 0),
                        new Pose2d(-50, 48),
                        new Pose2d(-50, 36 - 7.5 * vuMarkMap.get(vuMark)),
                        new Pose2d(-60, 36 - 7.5 * vuMarkMap.get(vuMark), 0)
                ));
            case NEAR_RED:
                return Path.createFromPoses(Arrays.asList(
                        new Pose2d(48, 48, 0),
                        new Pose2d(12 + 7.5 * vuMarkMap.get(vuMark), 48),
                        new Pose2d(12 + 7.5 * vuMarkMap.get(vuMark), 60, Math.PI / 2)
                ));
        }
        return null;
    }
}
