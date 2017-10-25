package com.acmerobotics.library.dashboard.config;

import java.util.HashMap;
import java.util.Map;

/**
 * Created by ryanbrott on 10/13/17.
 */

public class CustomOption extends Option {
    protected Map<String, Option> options;

    public CustomOption() {
        super(OptionType.CUSTOM);

        this.options = new HashMap<>();
    }

    public void addOption(String name, Option option) {
        this.options.put(name, option);
    }

    @Override
    protected Object getValue() {
        return options;
    }

    @Override
    protected void setValue(Object object) {
        this.options = (Map<String, Option>) object;
    }
}
