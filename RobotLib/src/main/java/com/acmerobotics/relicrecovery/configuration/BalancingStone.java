package com.acmerobotics.relicrecovery.configuration;

import com.acmerobotics.library.localization.Vector2d;

public enum BalancingStone {
    NEAR_BLUE(0, AllianceColor.BLUE, new Vector2d(48, -48)),
    FAR_BLUE(1, AllianceColor.BLUE, new Vector2d(-24, -48)),
    NEAR_RED(2, AllianceColor.RED, new Vector2d(48, 48)),
    FAR_RED(3, AllianceColor.RED, new Vector2d(-24, 48));

    private int index;
    private AllianceColor allianceColor;
    private Vector2d position;

    BalancingStone(int i, AllianceColor color, Vector2d position) {
        this.index = i;
        this.allianceColor = color;
        this.position = position;
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

    public Cryptobox getCryptobox() {
        return Cryptobox.fromIndex(index);
    }

    public Vector2d getPosition() {
        return position;
    }
}
