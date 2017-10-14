package com.acmerobotics.library.dashboard.config;

import android.util.Log;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import static com.acmerobotics.library.dashboard.RobotDashboard.TAG;

/**
 * @author Ryan
 */

public class OptionGroup {
    private String name;
    private Map<String, Option> options;

    public OptionGroup(String name) {
        this.name = name;
        this.options = new HashMap<>();
    }

    public String getName() {
        return name;
    }

    public void addOption(String name, Option option) {
        this.options.put(name, option);
    }

    public Map<String, Option> getOptions() {
        return options;
    }

    public JsonElement getAsJson() {
        JsonObject obj = new JsonObject();
        obj.add("name", new JsonPrimitive(name));
        JsonObject optionsObj = new JsonObject();
        for (Map.Entry<String, Option> entry : options.entrySet()) {
            optionsObj.add(entry.getKey(), entry.getValue().getJson());
        }
        obj.add("options", optionsObj);
        return obj;
    }

    public void updateFromJson(JsonElement json) {
        throw new UnsupportedOperationException();
    }

    public static OptionGroup createFromClass(Class<?> klass) {
        OptionGroup optionGroup = new OptionGroup(klass.getSimpleName());
        for (Field field : klass.getFields()) {
            optionGroup.addOption(field.getName(), createOptionFromClass(field, null));
        }
        return optionGroup;
    }

    public static Option createOptionFromClass(Field field, Object parent) {
        Class<?> klass = field.getType();
        System.out.printf("%s\t%b\t%b\n", klass.getSimpleName(), klass.isPrimitive(), klass.isEnum());
        if (klass.isPrimitive() || klass == String.class) {
            return new FieldOption(field, parent);
        } else if (klass.isEnum()) {
            List<?> enumConstants = Arrays.asList(klass.getEnumConstants());
            String[] values = new String[enumConstants.size()];
            for (int i = 0; i < values.length; i++) {
                values[i] = enumConstants.get(i).toString();
            }
            return new FieldOption(field, parent, values);
        }

        CustomOption option = new CustomOption();
        for (Field nestedField : klass.getFields()) {
            if (Modifier.isFinal(field.getModifiers())) {
                continue;
            }

            String name = nestedField.getName();
            try {
                option.addOption(name, createOptionFromClass(nestedField, field.get(parent)));
            } catch (IllegalAccessException e) {
                Log.w(TAG, e);
            }
        }

        return option;
    }
}
