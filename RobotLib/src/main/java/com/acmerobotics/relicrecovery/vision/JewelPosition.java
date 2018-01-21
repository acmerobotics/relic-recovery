package com.acmerobotics.relicrecovery.vision;

/**
 * @author Ryan
 */
public enum JewelPosition {
    RED_BLUE("Red / Blue"),
    BLUE_RED("Blue / Red"),
    UNKNOWN("Unknown");

    // this is a weird workaround; avoids self-reference errors
    static {
        RED_BLUE.opp = BLUE_RED;
        BLUE_RED.opp = RED_BLUE;
        UNKNOWN.opp = UNKNOWN;
    }

    private JewelPosition opp;
    private String str;

    JewelPosition(String str) {
        this.str = str;
    }

    public JewelPosition opposite() {
        return opp;
    }

    @Override
    public String toString() {
        return str;
    }
}
