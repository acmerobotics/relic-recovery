package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.loops.Loop;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class PoseEstimator implements Loop {
    private MecanumDrive drive;
    private double[] lastRotations;
    private Pose2d pose;

    public PoseEstimator(MecanumDrive drive, Pose2d startingPose) {
        this.drive = drive;
        this.pose = startingPose;
    }

    @Override
    public synchronized void onLoop(long timestamp) {
        if (lastRotations == null) {
            lastRotations = drive.getRotations();
        } else {
            double[] rotations = drive.getRotations();
            double[] rotationDeltas = new double[rotations.length];
            for (int i = 0; i < rotationDeltas.length; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Pose2d poseDelta = drive.getPoseDelta(rotationDeltas);
            pose.add(poseDelta);

            lastRotations = rotations;
        }
    }

    public synchronized Pose2d getPose() {
        return pose.copy();
    }

    public synchronized void setPose(Pose2d pose) {
        this.pose = pose;
    }
}
