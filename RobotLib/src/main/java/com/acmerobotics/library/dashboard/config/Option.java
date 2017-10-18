package com.acmerobotics.library.dashboard.config;

import android.util.Log;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonPrimitive;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Map;

import static com.acmerobotics.library.dashboard.RobotDashboard.TAG;

/**
 * Created by ryanbrott on 10/13/17.
 */

public abstract class Option {
    private OptionType type;

    public Option(OptionType type) {
        this.type = type;
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
            return new JsonPrimitive(getValue().toString());
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

    public JsonElement getSchemaJson() {
        if (type == OptionType.CUSTOM) {
            JsonObject obj = new JsonObject();
            Map<String, Option> values = (Map<String, Option>) getValue();
            for (Map.Entry<String, Option> value : values.entrySet()) {
                obj.add(value.getKey(), value.getValue().getSchemaJson());
            }
            return obj;
        } else if (type == OptionType.ENUM) {
            JsonObject obj = new JsonObject();
            obj.add("type", new JsonPrimitive(type.stringVal));
            JsonArray values = new JsonArray();
            for (Object enumConstant : getValue().getClass().getEnumConstants()) {
                values.add(new JsonPrimitive(enumConstant.toString()));
            }
            obj.add("values", values);
            return obj;
        } else if (type == OptionType.DOUBLE) {
            return new JsonPrimitive((double) getValue());
        } else if (type == OptionType.INT) {
            return new JsonPrimitive((int) getValue());
        } else {
            return new JsonPrimitive((boolean) getValue());
        }
    }

    protected abstract Object getValue();
    protected abstract void setValue(Object object);

    public static Option createFromClass(Field field, Object parent) {
        Class<?> klass = field.getType();
        System.out.printf("%s\t%b\t%b\n", klass.getSimpleName(), klass.isPrimitive(), klass.isEnum());
        if (klass.isPrimitive() || klass.isEnum() || klass == String.class) {
            return new FieldOption(field, parent);
        }

        CustomOption option = new CustomOption();
        for (Field nestedField : klass.getFields()) {
            if (Modifier.isFinal(field.getModifiers())) {
                continue;
            }

            String name = nestedField.getName();
            try {
                option.addOption(name, createFromClass(nestedField, field.get(parent)));
            } catch (IllegalAccessException e) {
                Log.w(TAG, e);
            }
        }

        return option;
    }
}
