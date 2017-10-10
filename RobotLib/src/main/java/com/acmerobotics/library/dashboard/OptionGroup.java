package com.acmerobotics.library.dashboard;

import android.content.SharedPreferences;
import android.util.Log;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;
import com.google.gson.JsonObject;
import com.google.gson.JsonParser;
import com.google.gson.JsonPrimitive;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import java.lang.reflect.Field;
import java.lang.reflect.Modifier;
import java.util.Arrays;
import java.util.List;

/**
 * @author Ryan
 */

public class OptionGroup {
    private Class<?> klass;
    private boolean persist;
    private String prefKey, name;
    private SharedPreferences prefs;
    private SharedPreferences.Editor editor;

    public OptionGroup(Class<?> klass, SharedPreferences prefs) {
        this(klass, klass.getSimpleName(), prefs);
    }

    public OptionGroup(Class<?> klass, String name, SharedPreferences prefs) {
        this.klass = klass;
        this.name = name;

        if (klass.isAnnotationPresent(Persist.class)) {
            this.prefs = prefs;
            this.editor = prefs.edit();

            this.prefKey = klass.getAnnotation(Persist.class).value();
            String jsonString = prefs.getString(prefKey, null);
            if (jsonString != null) {
                try {
                    updateFromJson(new JsonParser().parse(jsonString));
                } catch (IllegalStateException e) {
                    editor.putString(this.prefKey, null);
                    editor.commit();
                }
            }
            this.persist = true;
        } else {
            this.persist = false;
        }
    }

    public String getName() {
        return name;
    }

    public JsonElement getAsJson() {
        try {
            JsonObject obj = new JsonObject();
            obj.add("name", new JsonPrimitive(name));
            JsonArray options = new JsonArray();
            for (Field f : klass.getFields()) {
                if (Modifier.isFinal(f.getModifiers())) {
                    continue;
                }
                JsonObject option = new JsonObject();
                option.add("name", new JsonPrimitive(f.getName()));
                Class<?> type = f.getType();
                switch (type.getSimpleName()) {
                    case "boolean":
                        option.add("type", new JsonPrimitive("boolean"));
                        option.add("value", new JsonPrimitive(f.getBoolean(null)));
                        break;
                    case "int":
                        option.add("type", new JsonPrimitive("int"));
                        option.add("value", new JsonPrimitive(f.getInt(null)));
                        break;
                    case "double":
                        option.add("type", new JsonPrimitive("double"));
                        option.add("value", new JsonPrimitive(f.getDouble(null)));
                        break;
                    case "String":
                        option.add("type", new JsonPrimitive("string"));
                        option.add("value", new JsonPrimitive(f.get(null).toString()));
                        break;
                    default:
                        if (type.isEnum()) {
                            option.add("type", new JsonPrimitive("enum"));
                            JsonArray values = new JsonArray();
                            List<?> enumConstants = Arrays.asList(type.getEnumConstants());
                            for (Object constant : enumConstants) {
                                values.add(new JsonPrimitive(constant.toString()));
                            }
                            option.add("values", values);
                            Object value = f.get(null);
                            if (value == null) {
                                option.add("value", new JsonPrimitive(0));
                            } else {
                                option.add("value", new JsonPrimitive(enumConstants.indexOf(value)));
                            }
                        } else if (type == PIDCoefficients.class) {
                            option.add("type", new JsonPrimitive("pid"));
                            PIDCoefficients coeffs = (PIDCoefficients) f.get(null);
                            JsonObject value = new JsonObject();
                            value.add("p", new JsonPrimitive(coeffs.p));
                            value.add("i", new JsonPrimitive(coeffs.i));
                            value.add("d", new JsonPrimitive(coeffs.d));
                            option.add("value", value);
                        }
                        break;
                }
                options.add(option);
            }
            obj.add("options", options);
            return options;
        } catch (IllegalAccessException e) {
            Log.w(RobotDashboard.TAG, e);
            return null;
        }
    }

    public void updateFromJson(JsonElement json) {
        try {
            JsonArray options = json.getAsJsonObject().get("options").getAsJsonArray();
            for (int i = 0; i < options.size(); i++) {
                JsonObject option = options.get(i).getAsJsonObject();
                Field f = klass.getField(option.get("name").getAsString());
                JsonElement value = option.get("value");
                String type = option.get("type").getAsString();
                switch (type) {
                    case "boolean":
                        f.setBoolean(null, value.getAsBoolean());
                        break;
                    case "int":
                        f.setInt(null, value.getAsInt());
                        break;
                    case "double":
                        f.setDouble(null, value.getAsDouble());
                        break;
                    case "string":
                        f.set(null, value.getAsString());
                        break;
                    case "enum":
                        int index = value.getAsInt();
                        f.set(null, f.getType().getEnumConstants()[index]);
                        break;
                    case "pid":
                        PIDCoefficients coeffs = (PIDCoefficients) f.get(null);
                        JsonObject valueObj = value.getAsJsonObject();
                        coeffs.p = valueObj.get("p").getAsDouble();
                        coeffs.i = valueObj.get("i").getAsDouble();
                        coeffs.d = valueObj.get("d").getAsDouble();
                        break;
                    default:
                        Log.w(RobotDashboard.TAG, "Unknown type received: " + type);
                }
            }
            if (this.persist) {
                editor.putString(this.prefKey, json.toString());
                editor.commit();
            }
        } catch (IllegalAccessException | NoSuchFieldException e) {
            Log.w(RobotDashboard.TAG, e);
        }
    }
}
