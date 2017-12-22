package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.loops.Looper;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class PositionEstimator {
    public static final double HISTORY_DURATION = 5; // in seconds

    private MecanumDrive drive;
    private double[] lastRotations;
    private volatile Vector2d currentPosition;
    private RingBuffer<TimestampedData<Vector2d>> positionDeltas;

    public PositionEstimator(MecanumDrive drive, Vector2d initialPosition) {
        this.drive = drive;
        this.currentPosition = initialPosition;
        // TODO: make this more flexible if we decide to change the loop hz
        positionDeltas = new RingBuffer<>((int) (HISTORY_DURATION / Looper.DEFAULT_LOOP_TIME));
        positionDeltas.add(new TimestampedData<>(initialPosition));
    }

    public synchronized void update(double timestamp) {
        if (lastRotations == null) {
            lastRotations = drive.getRotations();
        } else {
            double[] rotations = drive.getRotations();
            double[] rotationDeltas = new double[4];
            for (int i = 0; i < 4; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Vector2d robotPoseDelta = MecanumDrive.getPoseDelta(rotationDeltas).pos();
            Vector2d fieldPoseDelta = robotPoseDelta.rotated(drive.getHeading());

            currentPosition = currentPosition.added(fieldPoseDelta);

            positionDeltas.add(new TimestampedData<>(fieldPoseDelta, timestamp));

            lastRotations = rotations;
        }
    }

    public synchronized Vector2d getPosition() {
        return currentPosition;
    }

    public synchronized void setPosition(Vector2d newPosition) {
        currentPosition = newPosition;
    }

    public synchronized void updatePosition(Vector2d newPosition, double timestamp) {
        double x = newPosition.x(), y = newPosition.y();
        int i = 0;
        while (i < positionDeltas.size()) {
            TimestampedData<Vector2d> positionDelta = positionDeltas.get(i);
            if (positionDelta.timestamp < timestamp) {
                break;
            }
            x += positionDelta.data.x();
            y += positionDelta.data.y();
            i++;
        }
        // latency compensation
        currentPosition = newPosition.added(new Vector2d(x, y));
    }
}
