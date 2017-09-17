package com.acmerobotics.library.dashboard.message;

import com.google.gson.annotations.SerializedName;

import java.lang.reflect.Field;

/**
 * @author Ryan
 */

public class EnumUtil {
    public static <T extends  Enum> T fromValue(String value, Class<T> classOfT) {
        for (T type : classOfT.getEnumConstants()) {
            Field field = null;
            try {
                field = classOfT.getField(type.name());
            } catch (NoSuchFieldException e) {
                e.printStackTrace();
            }
            if ((field.isAnnotationPresent(SerializedName.class) &&
                    field.getAnnotation(SerializedName.class).value().equals(value)) ||
                    (field.getName().equals(value))) {
                return type;
            }
        }
        return null;
    }

}
