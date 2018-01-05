package com.acmerobotics.relicrecovery.loops;

import com.acmerobotics.library.util.TimestampedData;

/**
 * Created by ryanbrott on 10/28/17.
 */

public interface Loop {
    /** Uses the timescale described in {@link TimestampedData} */
    void onLoop(double timestamp, double dt);
}
