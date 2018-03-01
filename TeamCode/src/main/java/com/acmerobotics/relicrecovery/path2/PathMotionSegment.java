package com.acmerobotics.relicrecovery.path2;

public interface PathMotionSegment {
    PathMotionProfile profile();
    PathMotionProfile stoppingProfile(PathMotionProfile profile, double time);
}
