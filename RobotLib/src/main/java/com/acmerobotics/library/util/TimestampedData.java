package com.acmerobotics.library.util;

public class TimestampedData<T> {
    public final T data;
    /** timestamp in seconds on the System#nanoTime() timescale */
    public final double timestamp;

    public static double getCurrentTime() {
        return System.nanoTime() / Math.pow(10, 9);
    }

    public TimestampedData(T data) {
        this(data, getCurrentTime());
    }

    /** timestamp in seconds on the System#nanoTime() timescale */
    public TimestampedData(T data, double timestamp) {
        this.data = data;
        this.timestamp = timestamp;
    }
}
