package com.acmerobotics.relicrecovery.loops;

/**
 * Created by ryanbrott on 10/28/17.
 */

public interface Loop {
    /** Uses the timescale described in {@link com.acmerobotics.relicrecovery.drive.TimestampedData} */
    void onLoop(double timestamp, double dt);
}
