package com.acmerobotics.library.dashboard.util;

/**
 * Created by ryanbrott on 8/20/17.
 */

public interface ClassFilter {
    boolean shouldProcessClass(String className);
    void processClass(Class klass);
}
