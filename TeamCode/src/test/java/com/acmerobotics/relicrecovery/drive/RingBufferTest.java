package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.util.RingBuffer;

import org.junit.Before;
import org.junit.Test;

import static junit.framework.Assert.*;

public class RingBufferTest {
    private RingBuffer<Integer> buffer;

    @Before
    public void setup() {
        buffer = new RingBuffer<>(10);
        buffer.add(1);
        buffer.add(2);
    }


    @Test
    public void testSize() {
        assertEquals(2, buffer.size());
    }

    @Test
    public void testGet() {
        assertEquals(1, (int) buffer.get(1));
    }
}
