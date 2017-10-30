package com.acmerobotics.relicrecovery.drive;

import android.util.Log;

import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.loops.Loop;

/**
 * Created by ryanbrott on 10/28/17.
 */

public class PoseEstimator implements Loop {
    private MecanumDrive drive;
    private int[] lastPositions;
    private Pose2d pose;

    public PoseEstimator(MecanumDrive drive, Pose2d startingPose) {
        this.drive = drive;
        this.pose = startingPose;
    }

    @Override
    public synchronized void onLoop(long timestamp) {
        if (lastPositions == null) {
            lastPositions = drive.getPositions();
        } else {
            int[] positions = drive.getPositions();
            int[] positionDeltas = new int[positions.length];
            for (int i = 0; i < positions.length; i++) {
                positionDeltas[i] = positions[i] - lastPositions[i];
            }

            Pose2d poseDelta = MecanumDrive.getDelta(positionDeltas);

            Log.i("PoseEstimator", poseDelta.toString());

            pose.add(poseDelta);

            lastPositions = positions;
        }
    }

    public synchronized Pose2d getPose() {
        return pose.copy();
    }

    public synchronized void setPose(Pose2d pose) {
        this.pose = pose;
    }
}
