package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.relicrecovery.localization.Pose2d;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class PoseEstimator {
    private MecanumDrive drive;
    private double[] lastRotations;
    private Pose2d pose;

    public PoseEstimator(MecanumDrive drive, Pose2d initialPose) {
        this.drive = drive;
        this.pose = initialPose;
    }

    public void update(long timestamp) {
        if (lastRotations == null) {
            lastRotations = drive.getRotations();
        } else {
            double[] rotations = drive.getRotations();
            double[] rotationDeltas = new double[4];
            for (int i = 0; i < 4; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Pose2d poseDelta = MecanumDrive.getPoseDelta(rotationDeltas);

            pose = new Pose2d(pose.pos().added(poseDelta.pos().rotated(drive.getHeading())), drive.getHeading());

            lastRotations = rotations;
        }
    }

    public Pose2d getPose() {
        return pose.copy();
    }

    public void setPose(Pose2d pose) {
        this.pose = pose;
    }
}
