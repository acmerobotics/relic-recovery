package com.acmerobotics.relicrecovery.configuration;

import com.acmerobotics.splinelib.Vector2d;

public enum Cryptobox {
    NEAR_BLUE(0, AllianceColor.BLUE, new Vector2d(12, -72)),
    FAR_BLUE(1, AllianceColor.BLUE, new Vector2d(-72, -36)),
    NEAR_RED(2, AllianceColor.RED, new Vector2d(12, 72)),
    FAR_RED(3, AllianceColor.RED, new Vector2d(-72, 36));

    private int index;
    private AllianceColor allianceColor;
    private Vector2d pos;

    Cryptobox(int i, AllianceColor color, Vector2d pos) {
        this.index = i;
        this.allianceColor = color;
        this.pos = pos;
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

    public Vector2d getPosition() {
        return pos;
    }

    public BalancingStone getBalancingStone() {
        return BalancingStone.fromIndex(index);
    }
}
