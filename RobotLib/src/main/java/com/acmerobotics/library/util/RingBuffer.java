package com.acmerobotics.library.util;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class RingBuffer<T> {
    private T[] buffer;
    private int capacity, nextPos, available;

    public RingBuffer(int capacity) {
        this.capacity = capacity;
        buffer = (T[]) new Object[capacity];
    }

    public int size() {
        return available;
    }

    public T get(int i) {
        int pos = (nextPos - i - 1 + capacity) % capacity;
        return buffer[pos];
    }

    public void add(T value) {
        buffer[nextPos] = value;
        nextPos = (nextPos + 1) % capacity;
        available = Math.min(available + 1, capacity);
    }
}
