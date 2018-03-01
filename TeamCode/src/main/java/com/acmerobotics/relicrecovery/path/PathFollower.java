package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.motion.PIDFController;

@Config
public class PathFollower {
    public static double NOMINAL_POWER = 0;

    private PIDFController headingController, axialController;
    private PIDFController lateralController;
    private Path path;
    private double pathStartTimestamp;

    private double headingError, headingUpdate;
    private double axialError, axialUpdate;
    private double lateralError, lateralUpdate;

    private Pose2d pose, poseVelocity, poseAcceleration;

    public PathFollower(PIDFCoefficients headingCoeff, PIDFCoefficients axialCoeff, PIDFCoefficients lateralCoeff) {
        headingController = new PIDFController(headingCoeff);
        headingController.setInputBounds(-Math.PI, Math.PI);

        axialController = new PIDFController(axialCoeff);

        lateralController = new PIDFController(lateralCoeff);
    }

    public Path getPath() {
        return path;
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
        this.pathStartTimestamp = TimestampedData.getCurrentTime();

        headingController.reset();
        axialController.reset();
        lateralController.reset();
    }

    public boolean isFollowingPath() {
        return isFollowingPath(TimestampedData.getCurrentTime());
    }

    public boolean isFollowingPath(double timestamp) {
        return path != null && (timestamp - pathStartTimestamp) < path.duration();
    }

    public Pose2d update(Pose2d estimatedPose) {
        return update(estimatedPose, TimestampedData.getCurrentTime());
    }

    /**
     * Update the drive controls
     * @param estimatedPose current robot pose
     * @param timestamp current time in seconds
     * @return the desired velocity
     */
    public Pose2d update(Pose2d estimatedPose, double timestamp) {
        double time = timestamp - pathStartTimestamp;
        if (time > path.duration()) {
            return new Pose2d(0, 0, 0);
        }

        // all field coordinates
        pose = path.getPose(time);
        poseVelocity = path.getVelocity(time);
        poseAcceleration = path.getAcceleration(time);

        MotionState headingState = new MotionState(pose.heading(), poseVelocity.heading(), poseAcceleration.heading(), 0, 0);
        headingController.setSetpoint(headingState);
        headingError = headingController.getPositionError(estimatedPose.heading());
        headingUpdate = headingController.update(headingError, time);

        Vector2d fieldError = estimatedPose.pos().added(pose.pos().negated());
        Vector2d robotError = fieldError.rotated(-estimatedPose.heading());

        axialError = robotError.x();
        lateralError = robotError.y();

        Vector2d fieldVelocity = new Vector2d(poseVelocity.x(), poseVelocity.y());
        Vector2d robotVelocity = fieldVelocity.rotated(-estimatedPose.heading());

        Vector2d fieldAcceleration = new Vector2d(poseAcceleration.x(), poseAcceleration.y());
        Vector2d robotAcceleration = fieldAcceleration.rotated(-estimatedPose.heading());

        MotionState axialState = new MotionState(pose.x(), robotVelocity.x(), robotAcceleration.x(), 0, 0);
        axialController.setSetpoint(axialState);
        axialUpdate = axialController.update(axialError, time);

        MotionState lateralState = new MotionState(pose.y(), robotVelocity.y(), robotAcceleration.y(), 0, 0);
        lateralController.setSetpoint(lateralState);
        lateralUpdate = lateralController.update(lateralError, time);

        if (Math.abs(axialUpdate) < NOMINAL_POWER) {
            axialUpdate = 0;
        }

        if (Math.abs(lateralUpdate) < NOMINAL_POWER) {
            lateralUpdate = 0;
        }

        if (Math.abs(headingUpdate) < NOMINAL_POWER) {
            headingUpdate = 0;
        }

        return new Pose2d(axialUpdate, lateralUpdate, headingUpdate);
    }
}
