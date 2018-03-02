package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;

public class PointTurn implements TrajectorySegment {
    private Pose2d startPose;
    private double endHeading;
    private MotionProfile profile;

    public PointTurn(Pose2d startPose, double endHeading) {
        this.startPose = startPose;
        this.endHeading = endHeading;
        double startHeading = startPose.heading(), displacement;
        displacement = endHeading - startHeading;
        if (endHeading < startHeading) {
            displacement -= 2 * Math.PI;
        }
        MotionState start = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(displacement, 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, MecanumDrive.POINT_TURN_CONSTRAINTS);
    }

    @Override
    public double duration() {
        return profile.end().t;
    }

    @Override
    public Pose2d start() {
        return startPose;
    }

    @Override
    public Pose2d end() {
        return new Pose2d(startPose.pos(), endHeading);
    }

    @Override
    public Pose2d getPose(double time) {
        return new Pose2d(startPose.pos(), Angle.norm(profile.get(time).x));
    }

    @Override
    public Pose2d getVelocity(double time) {
        return new Pose2d(0, 0, profile.get(time).v);
    }

    @Override
    public Pose2d getAcceleration(double time) {
        return new Pose2d(0, 0, profile.get(time).a);
    }

    @Override
    public void trimRemainingDistance(double time) {
        // do nothing; TODO: is there something better to do here?
    }
}
