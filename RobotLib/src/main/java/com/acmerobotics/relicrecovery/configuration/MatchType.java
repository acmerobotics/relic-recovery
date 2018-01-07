package com.acmerobotics.relicrecovery.configuration;

/**
 * Created by ryanbrott on 11/12/17.
 */
public enum MatchType {
    PRACTICE(0),
    QUALIFYING(1),
    SEMIFINAL(2),
    FINAL(3);

    private int index;

    MatchType(int i) {
        index = i;
    }

    public int getIndex() {
        return index;
    }

    public static MatchType fromIndex(int i) {
        for (MatchType type : MatchType.values()) {
            if (type.getIndex() == i) {
                return type;
            }
        }
        return null;
    }
}
