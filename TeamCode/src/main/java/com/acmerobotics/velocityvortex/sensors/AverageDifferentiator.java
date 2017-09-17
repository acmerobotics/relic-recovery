package com.acmerobotics.velocityvortex.sensors;


/**
 * @author Ryan
 */

public class AverageDifferentiator {

    public static final int STARTING_CAPACITY = 50;
    public static final int CAPACITY_INCR = 25;

    private long[] times;
    private double[] values;
    private int capacity, now, interval, timeout;
    private double lastVel;

    public AverageDifferentiator(int interval) {
        this(interval, 0);
    }

    public AverageDifferentiator(int interval, int timeout) {
        capacity = STARTING_CAPACITY;
        times = new long[capacity];
        values = new double[capacity];
        now = -1;
        this.interval = interval;
        this.timeout = timeout;
    }

    public int getInterval() {
        return interval;
    }

    public double update(double value) {
        return update(System.currentTimeMillis(), value);
    }

    public double update(long time, double value) {
        if (now != -1 && value == values[now] && (timeout != 0 && (time - times[now]) < timeout)) {
            return lastVel;
        }
        int i = now;
        double vel = 0;
        while (true) {
            i--;
            if (i < 0) {
                i += capacity;
            }
            long pastTime = times[i];

            if (pastTime == 0) {
                // didn't find a value
                break;
            }

            if (i == now) {
                // need more room
                System.out.println("expanding");
                expand(CAPACITY_INCR);
                break;
            }

            if ((time - pastTime) > interval) {
                // found the value
                double dt = (time - pastTime);
                if (dt != 0) {
                    vel = (value - values[i]) / dt;
                }
                break;
            }
        }
        add(time, value);
        lastVel = vel;
        return lastVel;
    }

    public double getLastDerivative() {
        return lastVel;
    }

    private void expand(int incr) {
        int newCapacity = capacity + incr;
        long[] newTimes = new long[newCapacity];
        double[] newValues = new double[newCapacity];

        System.arraycopy(times, 0, newTimes, 0, now + 1);
        System.arraycopy(times, now + 1, newTimes, now + 1 + incr, capacity - now - 1);

        System.arraycopy(values, 0, newValues, 0, now + 1);
        System.arraycopy(values, now + 1, newValues, now + 1 + incr, capacity - now - 1);

        capacity = newCapacity;
        times = newTimes;
        values = newValues;
    }

    public int getCapacity() {
        return capacity;
    }

    public void add(long time, double value) {
        now = (now + 1) % capacity;
        times[now] = time;
        values[now] = value;
    }
}
