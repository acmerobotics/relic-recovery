package com.acmerobotics.library.dashboard.config;

/**
 * Created by ryanbrott on 10/25/17.
 */

public interface ValueProvider<T> {
    T get();
    void set(T value);
}
