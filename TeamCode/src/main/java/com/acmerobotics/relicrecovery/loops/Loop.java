package com.acmerobotics.relicrecovery.loops;

/**
 * Created by ryanbrott on 10/28/17.
 */

public interface Loop {
    /**
     * Called periodically by {@link Looper}
     * @param timestamp starting timestamp on {@link com.acmerobotics.library.util.TimestampedData}'s
     *                  timescale
     * @param dt time since the last loop
     */
    void onLoop(double timestamp, double dt);
}
