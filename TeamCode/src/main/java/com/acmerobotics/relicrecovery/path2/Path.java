package com.acmerobotics.relicrecovery.path2;

import com.acmerobotics.library.localization.Pose2d;

import java.util.ArrayList;
import java.util.List;

public class Path {
    private List<PathMotionSegment> motionSegments;
    private List<PathMotionProfile> motionProfiles;

    public Path() {
        this(new ArrayList<>());
    }

    public Path(List<PathMotionSegment> motionSegments) {
        this.motionSegments = new ArrayList<>();
        this.motionProfiles = new ArrayList<>();
        for (PathMotionSegment motionSegment : motionSegments) {
            addSegment(motionSegment);
        }
    }

    public void addSegment(PathMotionSegment pathMotionSegment) {
        motionSegments.add(pathMotionSegment);
        motionProfiles.add(pathMotionSegment.profile());
    }

    public double duration() {
        double duration = 0;
        for (PathMotionProfile motionProfile : motionProfiles) {
            duration += motionProfile.duration();
        }
        return duration;
    }

    private PathMotionProfile endProfile() {
        return motionProfiles.get(motionProfiles.size() - 1);
    }

    public Pose2d getPose(double time) {
        for (PathMotionProfile profile : motionProfiles) {
            if (time <= profile.duration()) {
                return profile.getPose(time);
            }
            time -= profile.duration();
        }
        return endProfile().end();
    }

    public Pose2d getVelocity(double time) {
        for (PathMotionProfile profile : motionProfiles) {
            if (time <= profile.duration()) {
                return profile.getVelocity(time);
            }
            time -= profile.duration();
        }
        return endProfile().getVelocity(endProfile().duration());
    }

    public Pose2d getAcceleration(double time) {
        for (PathMotionProfile profile : motionProfiles) {
            if (time <= profile.duration()) {
                return profile.getAcceleration(time);
            }
            time -= profile.duration();
        }
        return endProfile().getAcceleration(endProfile().duration());
    }
}
