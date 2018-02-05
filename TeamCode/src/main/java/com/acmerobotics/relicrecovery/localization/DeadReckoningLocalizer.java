package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

public class DeadReckoningLocalizer implements Localizer {
    protected MecanumDrive drive;

    private Vector2d estimatedPosition;
    private double[] lastRotations;

    public DeadReckoningLocalizer(MecanumDrive drive) {
        this.drive = drive;
        this.lastRotations = new double[4];
    }

    @Override
    public Vector2d update() {
        double[] rotations = drive.getMotorRotations();
        if (lastRotations != null) {
            double[] rotationDeltas = new double[4];
            for (int i = 0; i < 4; i++) {
                rotationDeltas[i] = rotations[i] - lastRotations[i];
            }

            Vector2d robotPoseDelta = MecanumDrive.getPoseDelta(rotationDeltas).pos();
            Vector2d fieldPoseDelta = robotPoseDelta.rotated(drive.getHeading());

            estimatedPosition = estimatedPosition.added(fieldPoseDelta);
        }
        return estimatedPosition;
    }

    @Override
    public void setEstimatedPosition(Vector2d position) {
        estimatedPosition = position;
    }
}
