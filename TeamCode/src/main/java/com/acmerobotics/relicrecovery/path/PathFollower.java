package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.motion.PIDFController;
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

    public PathFollower(MecanumDrive drive, PIDFCoefficients headingCoeff, PIDFCoefficients axialCoeff, PIDCoefficients lateralCoeff) {
        this.drive = drive;

        headingController = new PIDFController(headingCoeff);
        headingController.setInputBounds(-Math.PI / 2, Math.PI / 2);
        headingController.setOutputBounds(-1, 1);

        axialController = new PIDFController(axialCoeff);
        axialController.setOutputBounds(-1, 1);

        lateralController = new PIDController(lateralCoeff);
        lateralController.setOutputBounds(-1, 1);
    }

    public void follow(Path path) {
        this.path = path;
        this.pathStartTimestamp = System.currentTimeMillis();

        headingController.reset();
        axialController.reset();
        lateralController.reset();
    }

    /**
     * Update the drive controls
     * @param robotPose current robot pose
     * @param timestamp current time in ms
     * @return true if the path is finished
     */
    public boolean update(Pose2d robotPose, long timestamp) {
        double time = (pathStartTimestamp - timestamp) / 1000.0;
        if (time > path.duration()) {
            drive.setVelocity(new Vector2d(0, 0), 0);
            return false;
        }

        Pose2d pose = path.getPose(time);
        Pose2d poseVelocity = path.getPoseVelocity(time);
        Pose2d poseAcceleration = path.getPoseAcceleration(time);

        MotionState headingState = new MotionState(pose.heading(), poseVelocity.heading(), poseAcceleration.heading(), 0, 0);
        headingController.setSetpoint(headingState);
        double headingError = headingController.getPositionError(robotPose.heading());
        double headingUpdate = headingController.update(headingError, time);

        Vector2d fieldError = robotPose.pos().add(pose.pos().negated());
        Vector2d robotError = fieldError.rotated(robotPose.heading());

        System.out.println(fieldError);
        System.out.println(robotError);

        double lateralError = robotError.x();
        double axialError = robotError.y();

        MotionState axialState = new MotionState(pose.x(), poseVelocity.x(), poseAcceleration.x(), 0, 0);
        axialController.setSetpoint(axialState);
        double axialUpdate = axialController.update(axialError, time);

        lateralController.setSetpoint(pose.y());
        double lateralUpdate = lateralController.update(lateralError, time);

        drive.setVelocity(new Vector2d(axialUpdate, lateralUpdate), headingUpdate);

        return false;
    }
}
