package com.acmerobotics.relicrecovery.vision;

/**
 * Created by ryanbrott on 1/8/18.
 */

public enum Detector {
    RED_CRYPTOBOX("Red Cryptobox"),
    BLUE_CRYPTOBOX("Blue Cryptobox"),
    JEWEL("Jewel");

    private String name;

    Detector(String name) {
        this.name = name;
    }

    public String getName() {
        return name;
    }

    public static Detector fromName(String name) {
        for (Detector detector : values()) {
            if (detector.name.equals(name)) {
                return detector;
            }
        }
        return null;
    }
}
