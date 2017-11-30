package com.acmerobotics.relicrecovery.loops;

/**
 * Created by ryanbrott on 10/28/17.
 */

public interface Loop {
    void loop(long timestamp, long dt);
}
