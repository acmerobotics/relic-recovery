package com.acmerobotics.library.dashboard.config;

import android.util.Log;

import java.lang.reflect.Field;

import static com.acmerobotics.library.dashboard.RobotDashboard.TAG;

/**
 * Created by ryanbrott on 10/13/17.
 */

public class FieldProvider<T> implements ValueProvider<T> {
    private Field field;
    private Object parent;

    public FieldProvider(Field field, Object parent) {
        this.field = field;
        this.parent = parent;
    }

    @Override
    public T get() {
        try {
            return (T) field.get(parent);
        } catch (IllegalAccessException e) {
            Log.w(TAG, e);
        }
        return null;
    }

    @Override
    public void set(T value) {
        try {
            field.set(parent, value);
        } catch (IllegalAccessException e) {
            Log.w(TAG, e);
        }
    }
}
