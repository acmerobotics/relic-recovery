package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.localization.Pose2d;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class PoseEstimator {
    private MecanumDrive drive;
    private double[] lastRotations;
    private volatile Pose2d pose;
    private RingBuffer<TimestampedData<Pose2d>> poseDeltaHistory;

    public PoseEstimator(MecanumDrive drive, Pose2d initialPose) {
        this.drive = drive;
        this.pose = initialPose;
        this.poseDeltaHistory = new RingBuffer<>(100); // 2 seconds (2000 / 20)
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

            Pose2d robotPoseDelta = MecanumDrive.getPoseDelta(rotationDeltas);
            Pose2d fieldPoseDelta = new Pose2d(robotPoseDelta.pos().rotated(drive.getHeading()), drive.getHeading() - pose.heading());

            poseDeltaHistory.add(new TimestampedData<>(fieldPoseDelta, timestamp));

            pose = pose.added(fieldPoseDelta);

            lastRotations = rotations;
        }
    }

    public Pose2d getPose() {
        return pose.copy();
    }

    public synchronized void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public synchronized void updatePose(Pose2d newPose, double timestamp) {
        TimestampedData<Pose2d> timestampedPoseDelta;
        int i = 0;
        double initialX = newPose.x(), initialY = newPose.y(), initialHeading = newPose.heading();
        do {
            if (i >= poseDeltaHistory.size()) {
                return;
            }
            timestampedPoseDelta = poseDeltaHistory.get(i);
            initialX += timestampedPoseDelta.data.x();
            initialY += timestampedPoseDelta.data.y();
            initialHeading += timestampedPoseDelta.data.heading();
        } while (timestampedPoseDelta.timestamp > timestamp);
        this.pose = new Pose2d(initialX, initialY, initialHeading);
    }
}
