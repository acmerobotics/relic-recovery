package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.localization.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Path {
    private List<PathMotionSegment> motionSegments;

    public Path() {
        this(new ArrayList<>());
    }

    public Path(List<PathMotionSegment> motionSegments) {
        this.motionSegments = motionSegments;
    }

    public double duration() {
        double duration = 0;
        for (PathMotionSegment motionSegment : motionSegments) {
            duration += motionSegment.duration();
        }
        return duration;
    }

    private PathMotionSegment endSegment() {
        return motionSegments.get(motionSegments.size() - 1);
    }

    public Pose2d getPose(double time) {
        for (PathMotionSegment motionSegment : motionSegments) {
            if (time <= motionSegment.duration()) {
                return motionSegment.getPose(time);
            }
            time -= motionSegment.duration();
        }
        return endSegment().end();
    }

    public Pose2d getVelocity(double time) {
        for (PathMotionSegment motionSegment : motionSegments) {
            if (time <= motionSegment.duration()) {
                return motionSegment.getVelocity(time);
            }
            time -= motionSegment.duration();
        }
        return endSegment().getVelocity(endSegment().duration());
    }

    public Pose2d getAcceleration(double time) {
        for (PathMotionSegment motionSegment : motionSegments) {
            if (time <= motionSegment.duration()) {
                return motionSegment.getAcceleration(time);
            }
            time -= motionSegment.duration();
        }
        return endSegment().getAcceleration(endSegment().duration());
    }

    public void trimRemainingDistance(double time) {
        for (int i = 0; i < motionSegments.size(); i++) {
            PathMotionSegment motionSegment = motionSegments.get(i);
            if (time <= motionSegment.duration()) {
                motionSegment.trimRemainingDistance(time);
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
