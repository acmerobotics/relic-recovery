package com.acmerobotics.library.dashboard.config;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 10/13/17.
 */

public class ConfigSchema {
    private List<OptionGroup> optionGroups;

    public ConfigSchema() {
        optionGroups = new ArrayList<>();
    }

    public void addOptionGroup(OptionGroup group) {
        optionGroups.add(group);
    }

    public static ConfigSchema createFromClasses(List<Class<?>> klasses) {
        ConfigSchema schema = new ConfigSchema();
        for (Class<?> klass : klasses) {
            schema.addOptionGroup(OptionGroup.createFromClass(klass));
        }
        return schema;
    }
}
