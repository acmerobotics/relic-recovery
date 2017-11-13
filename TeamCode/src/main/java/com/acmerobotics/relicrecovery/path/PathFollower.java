package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.motion.PIDFController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;

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
    private Telemetry telemetry;

    public PathFollower(MecanumDrive drive, Telemetry telemetry, PIDFCoefficients headingCoeff, PIDFCoefficients axialCoeff, PIDCoefficients lateralCoeff) {
        this.drive = drive;

        this.telemetry = telemetry;

        headingController = new PIDFController(headingCoeff);
        headingController.setInputBounds(-Math.PI, Math.PI);
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

    public boolean isFollowingPath() {
        return path != null && (System.currentTimeMillis() - pathStartTimestamp) / 1000.0 < path.duration();
    }

    /**
     * Update the drive controls
     * @param robotPose current robot pose
     * @param timestamp current time in ms
     * @return true if the path is finished
     */
    public boolean update(Pose2d robotPose, long timestamp) {
        double time = (timestamp - pathStartTimestamp) / 1000.0;
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

        Vector2d fieldError = robotPose.pos().added(pose.pos().negated());
        Vector2d robotError = fieldError.rotated(-robotPose.heading());

        double axialError = robotError.x();
        double lateralError = robotError.y();

        MotionState axialState = new MotionState(pose.x(), poseVelocity.x(), poseAcceleration.x(), 0, 0);
        axialController.setSetpoint(axialState);
        double axialUpdate = axialController.update(axialError, time);

        lateralController.setSetpoint(pose.y());
        double lateralUpdate = lateralController.update(lateralError, time);

        drive.setVelocity(new Vector2d(axialUpdate, lateralUpdate), headingUpdate);

        if (telemetry != null) {
            telemetry.addData("headingError", headingError);
            telemetry.addData("headingUpdate", headingUpdate);

            telemetry.addData("fieldError", fieldError);
            telemetry.addData("robotError", robotError);

            telemetry.addData("axialError", axialError);
            telemetry.addData("lateralError", lateralError);

            telemetry.addData("axialUpdate", axialUpdate);
            telemetry.addData("lateralUpdate", lateralUpdate);
        }

        return false;
    }
}
