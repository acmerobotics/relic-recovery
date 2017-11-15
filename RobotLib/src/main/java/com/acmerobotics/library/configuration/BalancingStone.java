package com.acmerobotics.library.configuration;

/**
 * Created by ryanbrott on 11/12/17.
 */

public enum BalancingStone {
    NEAR_BLUE(0, AllianceColor.BLUE),
    FAR_BLUE(1, AllianceColor.BLUE),
    NEAR_RED(2, AllianceColor.RED),
    FAR_RED(3, AllianceColor.RED);

    private int index;
    private AllianceColor allianceColor;

    BalancingStone(int i, AllianceColor color) {
        this.index = i;
        this.allianceColor = color;
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
}
