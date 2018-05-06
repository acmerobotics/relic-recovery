package com.acmerobotics.library.dashboard.util;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

public class TelemetryUtil {
    public static Map<String, Object> objectToMap(Object data) {
        Map<String, Object> map = new HashMap<>();
        for (Field field : data.getClass().getFields()) {
            try {
                Object value = field.get(data);
                map.put(field.getName(), value == null ? "null" : value);
            } catch (IllegalAccessException e) {
                android.util.Log.w("TelemetryUtil", e);
            }
        }
        return map;
    }
}
