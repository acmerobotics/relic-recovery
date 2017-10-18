package com.acmerobotics.library.dashboard.config;

import com.google.gson.JsonArray;
import com.google.gson.JsonElement;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 10/13/17.
 */

public class Configuration {
    private List<OptionGroup> optionGroups;

    public Configuration() {
        optionGroups = new ArrayList<>();
    }

    public void addOptionGroup(OptionGroup optionGroup) {
        this.optionGroups.add(optionGroup);
    }

    public JsonElement getJson() {
        JsonArray array = new JsonArray();
        for (OptionGroup group : optionGroups) {
            array.add(group.getJson());
        }
        return array;
    }

    public JsonElement getJsonSchema() {
        JsonArray array = new JsonArray();
        for (OptionGroup group : optionGroups) {
            array.add(group.getJsonSchema());
        }
        return array;
    }

    public static Configuration createFromClasses(List<Class<?>> klasses) {
        Configuration schema = new Configuration();
        for (Class<?> klass : klasses) {
            schema.addOptionGroup(OptionGroup.createFromClass(klass));
        }
        return schema;
    }
}
