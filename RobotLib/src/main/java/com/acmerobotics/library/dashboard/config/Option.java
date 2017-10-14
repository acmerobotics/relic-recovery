package com.acmerobotics.library.dashboard.config;

import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

import java.util.Arrays;
import java.util.List;
import java.util.Map;

/**
 * Created by ryanbrott on 10/13/17.
 */

public abstract class Option {
    private OptionType type;
    private String[] values;

    public Option(OptionType type) {
        this(type, null);
    }

    public Option(OptionType type, String[] values) {
        this.type = type;
        this.values = values;
    }

    public OptionType getType() {
        return type;
    }

    public JsonElement getJson() {
        if (type == OptionType.CUSTOM) {
            JsonObject obj = new JsonObject();
            Map<String, Option> values = (Map<String, Option>) getValue();
            for (Map.Entry<String, Option> value : values.entrySet()) {
                obj.add(value.getKey(), value.getValue().getJson());
            }
            return obj;
        } else if (type == OptionType.ENUM) {
            Object value = getValue();
            List<?> enumConstants = Arrays.asList(value.getClass().getEnumConstants());
            return new JsonPrimitive(enumConstants.indexOf(value));
        } else if (type == OptionType.DOUBLE) {
            return new JsonPrimitive((double) getValue());
        } else if (type == OptionType.INT) {
            return new JsonPrimitive((int) getValue());
        } else {
            return new JsonPrimitive((boolean) getValue());
        }
    }

    public void updateJson(JsonElement element) {
        if (type == OptionType.CUSTOM) {
            JsonObject obj = element.getAsJsonObject();
            Map<String, Option> values = (Map<String, Option>) getValue();
            for (Map.Entry<String, JsonElement> entry : obj.entrySet()) {
                values.get(entry.getKey()).updateJson(entry.getValue());
            }
        } else if (type == OptionType.ENUM) {
            setValue(getValue().getClass().getEnumConstants()[element.getAsInt()]);
        } else if (type == OptionType.DOUBLE) {
            setValue(element.getAsDouble());
        } else if (type == OptionType.INT) {
            setValue(element.getAsInt());
        } else {
            setValue(element.getAsBoolean());
        }
    }

    protected abstract Object getValue();
    protected abstract void setValue(Object object);

}
