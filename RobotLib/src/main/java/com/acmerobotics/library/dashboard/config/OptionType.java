package com.acmerobotics.library.dashboard.config;

/**
 * Created by ryanbrott on 10/13/17.
 */

public enum OptionType {
    BOOLEAN("boolean"),
    INT("int"),
    DOUBLE("double"),
    STRING("string"),
    ENUM("enum"),
    CUSTOM("custom");

    public final String stringVal;

    OptionType(String val) {
        this.stringVal = val;
    }

    public static OptionType fromClass(Class<?> klass) {
        if (klass == Boolean.class || klass == boolean.class) {
            return BOOLEAN;
        } else if (klass == Integer.class || klass == int.class) {
            return INT;
        } else if (klass == Double.class || klass == double.class) {
            return DOUBLE;
        } else if (klass == String.class) {
            return STRING;
        } else if (klass.isEnum()) {
            return ENUM;
        } else {
            return CUSTOM;
        }
    }
}
