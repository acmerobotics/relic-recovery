package com.acmerobotics.library.configuration;

import com.acmerobotics.library.localization.Pose2d;

/**
 * Created by ryanbrott on 11/12/17.
 */

public enum Cryptobox {
    NEAR_BLUE(0, AllianceColor.BLUE, new Pose2d(12, -72, Math.PI / 2)),
    FAR_BLUE(1, AllianceColor.BLUE, new Pose2d(-72, -36, 0)),
    NEAR_RED(2, AllianceColor.RED, new Pose2d(12, 72, - Math.PI / 2)),
    FAR_RED(3, AllianceColor.RED, new Pose2d(-72, 36, 0));

    private int index;
    private AllianceColor allianceColor;
    private Pose2d pose;

    Cryptobox(int i, AllianceColor color, Pose2d pose) {
        this.index = i;
        this.allianceColor = color;
        this.pose = pose;
    }

    public int getIndex() {
        return index;
    }

    public static Cryptobox fromIndex(int i) {
        for (Cryptobox color : Cryptobox.values()) {
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

    public BalancingStone getBalancingStone() {
        return BalancingStone.fromIndex(index);
    }
}
