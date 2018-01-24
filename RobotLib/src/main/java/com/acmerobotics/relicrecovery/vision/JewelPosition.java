package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.relicrecovery.configuration.AllianceColor;

public enum JewelPosition {
    RED_BLUE("Red / Blue", AllianceColor.RED, AllianceColor.BLUE),
    BLUE_RED("Blue / Red", AllianceColor.BLUE, AllianceColor.RED),
    UNKNOWN("Unknown", null, null);

    // this is a weird workaround; avoids self-reference errors
    static {
        RED_BLUE.opp = BLUE_RED;
        BLUE_RED.opp = RED_BLUE;
        UNKNOWN.opp = UNKNOWN;
    }

    private JewelPosition opp;
    private AllianceColor leftColor, rightColor;
    private String str;

    JewelPosition(String str, AllianceColor leftColor, AllianceColor rightColor) {
        this.str = str;
        this.leftColor = leftColor;
        this.rightColor = rightColor;
    }

    public JewelPosition opposite() {
        return opp;
    }

    public AllianceColor leftColor() {
        return leftColor;
    }

    public AllianceColor rightColor() {
        return rightColor;
    }

    @Override
    public String toString() {
        return str;
    }
}
