package com.acmerobotics.library.dashboard.config;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

import java.lang.reflect.Field;
import java.util.HashMap;
import java.util.Map;

/**
 * @author Ryan
 *
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

    public JsonElement getJson() {
        JsonObject obj = new JsonObject();
        obj.add("name", new JsonPrimitive(name));
        JsonObject optionsObj = new JsonObject();
        for (Map.Entry<String, Option> entry : options.entrySet()) {
            optionsObj.add(entry.getKey(), entry.getValue().getJson());
        }
        obj.add("options", optionsObj);
        return obj;
    }

    public void updateJson(JsonElement json) {
        throw new UnsupportedOperationException();
    }

    public JsonElement getJsonSchema() {
        JsonObject obj = new JsonObject();
        obj.add("name", new JsonPrimitive(name));
        JsonObject schemaObj = new JsonObject();
        for (Map.Entry<String, Option> entry : options.entrySet()) {
            schemaObj.add(entry.getKey(), entry.getValue().getSchemaJson());
        }
        obj.add("optionsSchema", schemaObj);
        return obj;
    }

    public static OptionGroup createFromClass(Class<?> klass) {
        String name = klass.getSimpleName();
        if (klass.isAnnotationPresent(Config.class)) {
            String altName = klass.getAnnotation(Config.class).value();
            if (altName.length() != 0) {
                name = altName;
            }
        }
        return createFromClass(klass, name);
    }

    public static OptionGroup createFromClass(Class<?> klass, String name) {
        OptionGroup optionGroup = new OptionGroup(name);
        for (Field field : klass.getFields()) {
            optionGroup.addOption(field.getName(), Option.createFromClass(field, null));
        }
        return optionGroup;
    }
}
