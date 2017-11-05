package com.acmerobotics.relicrecovery.loops;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class Looper extends Thread {
    private List<Loop> loops;
    private long loopMs;
    private boolean running;

    public Looper(long loopMs) {
        this.loopMs = loopMs;
        loops = new ArrayList<>();
    }

    @Override
    public void run() {
        this.running = true;
        while (running) {
            long startTime = System.currentTimeMillis();

            for (Loop loop : loops) {
                loop.onLoop(startTime);
            }

            while (System.currentTimeMillis() - startTime > loopMs) {
                try {
                    Thread.sleep(0, 250);
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                }
            }
        }
    }

    public void terminate() {
        this.running = false;
    }

    public void addLoop(Loop loop) {
        this.loops.add(loop);
    }
}
