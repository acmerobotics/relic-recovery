package com.acmerobotics.library.dashboard.util;

public interface ClassFilter {
    boolean shouldProcessClass(String className);
    void processClass(Class klass);
}
