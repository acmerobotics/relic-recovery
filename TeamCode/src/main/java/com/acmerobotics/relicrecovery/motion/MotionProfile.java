package com.acmerobotics.relicrecovery.motion;


import com.acmerobotics.relicrecovery.util.SuperArrayList;

/**
 * Created by kelly on 10/14/2017.
 *
 */

public class MotionProfile {

    private SuperArrayList<MotionSegment> segments;
    private MotionState end;

    public MotionProfile(MotionState start) {
        segments = new SuperArrayList<>();
        this.end = start;
    }

    protected MotionProfile(SuperArrayList<MotionSegment> segments) {
        this.segments = segments;
        this.end = segments.get(-1).end();
    }

    public void appendControl(double j, double t) {
        MotionState segmentStart = new MotionState(end.x, end.v, end.a, j, end.t);
        segments.add(new MotionSegment(segmentStart, t));
        end = segments.get(-1).end();
    }

    public void appendInfiniteJerkControl(double a, double t) {
        MotionState segmentStart = new MotionState(end.x, end.v, a, 0, end.t);
        segments.add(new MotionSegment(segmentStart, t));
        end = segments.get(-1).end();
    }

    public void appendProfile(MotionProfile other) {
        for (MotionSegment seg: other.segments) {
            appendControl(seg.start().j, seg.dt());
        }
    }

    public MotionState end() {
        return end;
    }

    public MotionState get(double t) {
        for (MotionSegment segment: segments) {
            if(segment.start().t <= t && t <= segment.end().t) {
                return segment.get(t);
            }
        }
        return null;
    }

    public MotionProfile trimBefore(double t) {
        SuperArrayList<MotionSegment> newSegments = new SuperArrayList<>();
        for (MotionSegment segment: segments) {
            if (segment.end().t < t) {
                continue;
            }
            if (segment.start().t < t) {
                newSegments.add(new MotionSegment(segment.get(t), segment.end().t - t));
            } else {
                newSegments.add(segment);
            }
        }
        return new MotionProfile(newSegments);
    }

    public MotionProfile trimAfter(double t) {
        SuperArrayList<MotionSegment> newSegments = new SuperArrayList<>();
        for (MotionSegment segment: segments) {
            if (segment.start().t > t) {
                continue;
            }
            if (segment.end().t > t) {
                newSegments.add(new MotionSegment(segment.start(), t - segment.start().t));
            } else {
                newSegments.add(segment);
            }
        }
        return new MotionProfile(newSegments);
    }

    public double timeAtPos(double pos) {
        for (MotionSegment seg: segments) {
            double t = seg.timeAtPos(pos);
            if (t != -1) return t;
        }
        return -1;
    }

    public SuperArrayList<MotionSegment> segments() {
        return segments;
    }

}
