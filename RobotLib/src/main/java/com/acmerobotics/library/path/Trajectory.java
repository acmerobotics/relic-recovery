package com.acmerobotics.library.path;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.path.parametric.CompositePath;
import com.acmerobotics.library.path.parametric.Marker;
import com.acmerobotics.library.path.parametric.ParametricPath;

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

    public double getMarkerTime(String name) {
        double duration = 0;
        for (int i = 0; i < motionSegments.size(); i++) {
            TrajectorySegment motionSegment = motionSegments.get(i);
            if (motionSegment instanceof ParametricSegment) {
                ParametricSegment parametricSegment = (ParametricSegment) motionSegment;
                MarkerSearchResult result = findMarkerOnPath(name, parametricSegment.path());
                if (result.found) {
                    return duration + parametricSegment.timeAtPos(result.position);
                }
            }
            duration += motionSegment.duration();
        }
        return Double.NaN;
    }

    private class MarkerSearchResult {
        public final boolean found;
        public final double position;

        private MarkerSearchResult(boolean found, double position) {
            this.found = found;
            this.position = position;
        }
    }

    private MarkerSearchResult findMarkerOnPath(String name, ParametricPath path) {
        if (path instanceof Marker) {
            Marker marker = (Marker) path;
            if (marker.getName().equals(name)) {
                return new MarkerSearchResult(true, 0);
            } else {
                return new MarkerSearchResult(false, 0);
            }
        } else if (path instanceof CompositePath) {
            CompositePath compositePath = (CompositePath) path;
            double markerPosition = 0;
            for (ParametricPath subPath : compositePath.segments()) {
                MarkerSearchResult result = findMarkerOnPath(name, subPath);
                if (result.found) {
                    return new MarkerSearchResult(true, result.position + markerPosition);
                }
                markerPosition += subPath.length();
            }
            return new MarkerSearchResult(false, compositePath.length());
        } else {
            return new MarkerSearchResult(false, path.length());
        }
    }
}
