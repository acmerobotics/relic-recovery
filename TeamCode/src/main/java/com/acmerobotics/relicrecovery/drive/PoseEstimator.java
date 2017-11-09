package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.relicrecovery.localization.Pose2d;

import java.sql.Time;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class PoseEstimator {

    public static class TimestampedPose2d {
        public final Pose2d pose;
        public final long timestamp;

        public TimestampedPose2d(Pose2d pose, long timestamp) {
            this.pose = pose;
            this.timestamp = timestamp;
        }
    }

    private MecanumDrive drive;
    private double[] lastRotations;
    private volatile Pose2d pose;
    private RingBuffer<TimestampedPose2d> poseDeltaHistory;

    public PoseEstimator(MecanumDrive drive, Pose2d initialPose) {
        this.drive = drive;
        this.pose = initialPose;
        this.poseDeltaHistory = new RingBuffer<>(100); // 2 seconds (2000 / 20)
    }

    public synchronized void update(long timestamp) {
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

            poseDeltaHistory.add(new TimestampedPose2d(fieldPoseDelta, timestamp));

            pose.add(fieldPoseDelta);

            lastRotations = rotations;
        }
    }

    public Pose2d getPose() {
        return pose.copy();
    }

    public synchronized void setPose(Pose2d pose) {
        this.pose = pose;
    }

    public synchronized void updatePose(Pose2d pose, long timestamp) {
        Pose2d newPose = new Pose2d(0, 0);
        TimestampedPose2d timestampedPoseDelta;
        int i = 0;
        do {
            if (i >= poseDeltaHistory.size()) {
                return;
            }
            timestampedPoseDelta = poseDeltaHistory.get(i);
            newPose.add(timestampedPoseDelta.pose);
        } while (timestampedPoseDelta.timestamp > timestamp);
        newPose.add(pose);
        this.pose = newPose;
    }
}
