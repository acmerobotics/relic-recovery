package com.acmerobotics.relicrecovery.motion;


import com.acmerobotics.relicrecovery.util.SuperArrayList;

/**
 * @author kellyrm
 * a list of continous motion segments
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

    /**
     * add on to the profile by applying a jerk for a length of time
     * @param j the jerk to apply
     * @param t the time to apply it for
     */
    public void appendControl(double j, double t) {
        MotionState segmentStart = new MotionState(end.x, end.v, end.a, j, end.t);
        segments.add(new MotionSegment(segmentStart, t));
        end = segments.get(-1).end();
    }

    /**
     * add on to the profile by applying an acceleration for a length of time
     * @param a the acceleration to apply
     * @param t the time to apply it for
     */
    public void appendInfiniteJerkControl(double a, double t) {
        MotionState segmentStart = new MotionState(end.x, end.v, a, 0, end.t);
        segments.add(new MotionSegment(segmentStart, t));
        end = segments.get(-1).end();
    }

    /**
     * appends another profile to this one
     * note: the segments themselves are not appended, rather the same controls are applied
     * @param other
     */
    public void appendProfile(MotionProfile other) {
        for (MotionSegment seg: other.segments) {
            appendControl(seg.start().j, seg.dt());
        }
    }

    public MotionState end() {
        return end;
    }

    /**
     * get the motion state at the specified time
     * @return the motion state if it is contained in the profile, otherwise null
     */
    public MotionState get(double t) {
        for (MotionSegment segment: segments) {
            if(segment.start().t <= t && t <= segment.end().t) {
                return segment.get(t);
            }
        }
        return null;
    }

    /**
     * remove all parts of the profile before a time
     * @param t time
     * @return the profile with parts before t removed
     */
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

    /**
     * remove all parts of the profile after a time
     * @param t time
     * @return the profile with parts after t removed
     */
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

    /**
     * find the next time the profile will be at a position
     *
     */
    public double timeAtPos(double pos) {
        for (MotionSegment seg: segments) {
            double t = seg.timeAtPos(pos);
            if (t != -1) return t;
        }
        return -1;
    }

    /**
     * remove all redundant motion segments from the profile
     */
    public void dereduntantify() {
        SuperArrayList<MotionSegment> newSegments = new SuperArrayList<>();
        for (MotionSegment seg: segments) {
            if (seg.dt() > 0) {
                newSegments.add(seg);
            }
        }
        segments = newSegments;
    }

    public SuperArrayList<MotionSegment> segments() {
        return segments;
    }

}
