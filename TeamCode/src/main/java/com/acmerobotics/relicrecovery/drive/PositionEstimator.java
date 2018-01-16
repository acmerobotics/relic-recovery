package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.RingBuffer;
import com.acmerobotics.library.util.TimestampedData;

/**
 * Created by ryanbrott on 10/28/17.
 */

@Deprecated
public class PositionEstimator {
    private MecanumDrive drive;
    private double[] lastRotations;
    private volatile Vector2d currentPosition;
    private RingBuffer<TimestampedData<Vector2d>> positionDeltas;

    private PositionEstimator(MecanumDrive drive, Vector2d initialPosition) {
        this.drive = drive;
        this.currentPosition = initialPosition;
        positionDeltas = new RingBuffer<>(500);
        positionDeltas.add(new TimestampedData<>(initialPosition));
    }

    public synchronized void update(double timestamp) {
        if (lastRotations == null) {
            lastRotations = drive.getMotorRotations();
        } else {
            double[] rotations = drive.getMotorRotations();
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
