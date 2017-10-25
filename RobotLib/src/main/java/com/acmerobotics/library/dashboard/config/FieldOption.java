package com.acmerobotics.library.dashboard.config;

import android.util.Log;

import java.lang.reflect.Field;

import static com.acmerobotics.library.dashboard.RobotDashboard.TAG;

/**
 * Created by ryanbrott on 10/13/17.
 */

public class FieldOption extends Option {
    private transient Field field;
    private transient Object parent;

    public FieldOption(Field field, Object parent) {
        super(OptionType.fromClass(field.getType()));

        this.field = field;
        this.parent = parent;
    }

    @Override
    public Object getValue() {
        try {
            return field.get(parent);
        } catch (IllegalAccessException e) {
            Log.w(TAG, e);
        }
        return null;
    }

    @Override
    public void setValue(Object o) {
        try {
            field.set(parent, o);
        } catch (IllegalAccessException e) {
            Log.w(TAG, e);
        }
    }
}
