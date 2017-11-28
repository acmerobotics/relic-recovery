package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.motion.PIDFController;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * @author Ryan
 */

@Config
public class PathFollower {

    private MecanumDrive drive;
    private PIDFController headingController, axialController;
    private PIDController lateralController;
    private Path path;
    private long pathStartTimestamp;

    private double headingError, headingUpdate;
    private double axialError, axialUpdate;
    private double lateralError, lateralUpdate;

    private Pose2d pose, poseVelocity, poseAcceleration;

    public PathFollower(MecanumDrive drive, PIDFCoefficients headingCoeff, PIDFCoefficients axialCoeff, PIDCoefficients lateralCoeff) {
        this.drive = drive;

        headingController = new PIDFController(headingCoeff);
        headingController.setInputBounds(-Math.PI, Math.PI);

        axialController = new PIDFController(axialCoeff);

        lateralController = new PIDController(lateralCoeff);
    }

    public double getHeadingError() {
        return headingError;
    }

    public double getHeadingUpdate() {
        return headingUpdate;
    }

    public double getAxialError() {
        return axialError;
    }

    public double getAxialUpdate() {
        return axialUpdate;
    }

    public double getLateralError() {
        return lateralError;
    }

    public double getLateralUpdate() {
        return lateralUpdate;
    }

    public Pose2d getPose() {
        return pose;
    }

    public Pose2d getPoseVelocity() {
        return poseVelocity;
    }

    public Pose2d getPoseAcceleration() {
        return poseAcceleration;
    }

    public void follow(Path path) {
        this.path = path;
        this.pathStartTimestamp = System.currentTimeMillis();

        headingController.reset();
        axialController.reset();
        lateralController.reset();
    }

    public boolean isFollowingPath() {
        return path != null && (System.currentTimeMillis() - pathStartTimestamp) / 1000.0 < path.duration();
    }

    /**
     * Update the drive controls
     * @param estimatedPose current robot pose
     * @param timestamp current time in ms
     * @return true if the path is finished
     */
    public boolean update(Pose2d estimatedPose, long timestamp) {
        double time = (timestamp - pathStartTimestamp) / 1000.0;
        if (time > path.duration()) {
            drive.setVelocity(new Vector2d(0, 0), 0);
            return false;
        }

        pose = path.getPose(time);
        poseVelocity = path.getPoseVelocity(time);
        poseAcceleration = path.getPoseAcceleration(time);

        MotionState headingState = new MotionState(pose.heading(), poseVelocity.heading(), poseAcceleration.heading(), 0, 0);
        headingController.setSetpoint(headingState);
        headingError = headingController.getPositionError(estimatedPose.heading());
        headingUpdate = headingController.update(headingError, time);

        Vector2d fieldError = estimatedPose.pos().added(pose.pos().negated());
        Vector2d robotError = fieldError.rotated(-estimatedPose.heading());

        axialError = robotError.x();
        lateralError = robotError.y();

        MotionState axialState = new MotionState(pose.x(), poseVelocity.x(), poseAcceleration.x(), 0, 0);
        axialController.setSetpoint(axialState);
        axialUpdate = axialController.update(axialError, time);

        lateralController.setSetpoint(pose.y());
        lateralUpdate = lateralController.update(lateralError, time);

        drive.internalSetVelocity(new Vector2d(axialUpdate, lateralUpdate), headingUpdate);

        return false;
    }
}
