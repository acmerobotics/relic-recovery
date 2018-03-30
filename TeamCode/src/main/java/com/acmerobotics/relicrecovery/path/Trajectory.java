package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Trajectory {
    private List<TrajectorySegment> motionSegments;

    public Trajectory() {
        this(new ArrayList<>());
    }

    public Trajectory(List<TrajectorySegment> motionSegments) {
        this.motionSegments = motionSegments;
    }

    public List<TrajectorySegment> segments() {
        return motionSegments;
    }

    public double duration() {
        double duration = 0;
        for (TrajectorySegment motionSegment : motionSegments) {
            duration += motionSegment.duration();
        }
        return duration;
    }

    private TrajectorySegment endSegment() {
        return motionSegments.get(motionSegments.size() - 1);
    }

    public Pose2d getPose(double time) {
        for (TrajectorySegment motionSegment : motionSegments) {
            if (time <= motionSegment.duration()) {
                return motionSegment.getPose(time);
            }
            time -= motionSegment.duration();
        }
        return endSegment().end();
    }

    public Pose2d getVelocity(double time) {
        for (TrajectorySegment motionSegment : motionSegments) {
            if (time <= motionSegment.duration()) {
                return motionSegment.getVelocity(time);
            }
            time -= motionSegment.duration();
        }
        return endSegment().getVelocity(endSegment().duration());
    }

    public Pose2d getAcceleration(double time) {
        for (TrajectorySegment motionSegment : motionSegments) {
            if (time <= motionSegment.duration()) {
                return motionSegment.getAcceleration(time);
            }
            time -= motionSegment.duration();
        }
        return endSegment().getAcceleration(endSegment().duration());
    }

    public void stopPrematurely(double time) {
        for (int i = 0; i < motionSegments.size(); i++) {
            TrajectorySegment motionSegment = motionSegments.get(i);
            if (time <= motionSegment.duration()) {
                motionSegment.stopPrematurely(time);
                while (motionSegments.size() > i + 1) {
                    motionSegments.remove(endSegment());
                }
                return;
            }
            time -= motionSegment.duration();
        }
    }

    public Pose2d start() {
        return motionSegments.get(0).start();
    }

    public Pose2d end() {
        return endSegment().end();
    }
}
