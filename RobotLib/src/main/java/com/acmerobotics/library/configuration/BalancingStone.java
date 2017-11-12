package com.acmerobotics.library.configuration;

import com.acmerobotics.library.localization.Pose2d;

/**
 * Created by ryanbrott on 11/12/17.
 */

public enum BalancingStone {
    NEAR_BLUE(0, AllianceColor.BLUE, new Pose2d(48, -48, Math.PI)),
    FAR_BLUE(1, AllianceColor.BLUE, new Pose2d(-24, -48, Math.PI / 2)),
    NEAR_RED(2, AllianceColor.RED, new Pose2d(48, 48, Math.PI)),
    FAR_RED(3, AllianceColor.RED, new Pose2d(-24, 48, -Math.PI / 2));

    private int index;
    private AllianceColor allianceColor;
    private Pose2d pose;

    BalancingStone(int i, AllianceColor color, Pose2d pose) {
        this.index = i;
        this.allianceColor = color;
        this.pose = pose;
    }

    public int getIndex() {
        return index;
    }

    public static BalancingStone fromIndex(int i) {
        for (BalancingStone color : BalancingStone.values()) {
            if (color.getIndex() == i) {
                return color;
            }
        }
        return null;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Cryptobox getCryptobox() {
        return Cryptobox.fromIndex(index);
    }
}
