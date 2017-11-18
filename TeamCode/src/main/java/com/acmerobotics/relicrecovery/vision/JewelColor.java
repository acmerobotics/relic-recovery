package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.configuration.AllianceColor;

/**
 * @author Ryan
 */
public enum JewelColor {
    RED(AllianceColor.RED),
    BLUE(AllianceColor.BLUE),
    UNKNOWN(null);

    // this is a weird workaround; avoids self-reference errors
    static {
        RED.opp = BLUE;
        BLUE.opp = RED;
        UNKNOWN.opp = UNKNOWN;
    }

    private JewelColor opp;
    private AllianceColor allianceColor;

    JewelColor(AllianceColor allianceColor) {
        this.allianceColor = allianceColor;
    }

    public JewelColor opposite() {
        return opp;
    }

    public AllianceColor getAllianceColor() {
        return allianceColor;
    }
}
