package com.acmerobotics.relicrecovery.motion;

import com.acmerobotics.relicrecovery.util.SuperArrayList;

/**
 * used to generate jerk-controlled seven-segment motion profiles given start and goal conditions, and some constraints
 *
 * like this:
 *        1   2   3      4        5   6   7
 *       ___                             ___
 * J:   |   |___     ___________     ___|   |
 *              |___|           |___|
 *
 * see <a href="https://pub.uni-bielefeld.de/download/1996788/2280502">this paper</a> (also in doc/Jerk_Limited_Trajectory_Planning.pdf)
 * someone should read page 4 and see if they have any clue for a implementation of the last step of the algorithm they outline
 * what I have now is more a hack than anything
 */

public class MotionProfileGenerator {

    //static class
    private MotionProfileGenerator() {}

    /**
     * make the motion profile!
     * @param start the initial state
     * @param goal the goal of the profile
     * @param constraints constraints the profile is subject to
     * @return the motion profile!
     */
    public static MotionProfile generateProfile(MotionState start, MotionGoal goal, MotionConstraints constraints) {

        //for simplicity, assume that the profile requires positive movement
        if (goal.pos < start.x) {
            //flip the profile and then flip the results
            return generateFlippedProfile(start, goal, constraints);
        }

        MotionProfile profile = new MotionProfile(start, constraints);

        //if we are headed away from the goal, first come to a stop
        if (start.v < 0) {
            double[] stoppingTimes = getDeltaVTimes(-start.v, constraints);
            profile.appendControl(constraints.maxJ, stoppingTimes[0]);
            profile.appendControl(0, stoppingTimes[1]);
            profile.appendControl(-constraints.maxJ, stoppingTimes[2]);
        }

        //neat lets go to the goal now
        //from this point on it only requires positive movement

        double dv = profile.end().v - goal.maxAbsV;
        //check if the profile requires us to slow down
        if (dv > 0) {
            //test if we can slow without breaking anything
            MotionProfile stoppingProfile = new MotionProfile(profile.end(), constraints);
            double[] stoppingTimes = getDeltaVTimes(dv, constraints);
            stoppingProfile.appendControl(-constraints.maxJ, stoppingTimes[0]);
            stoppingProfile.appendControl(0, stoppingTimes[1]);
            stoppingProfile.appendControl(constraints.maxJ, stoppingTimes[2]);
            double stoppingDistance = stoppingProfile.end().x;

            if (stoppingDistance > goal.pos) {
                //well, we can't stop now
                if (constraints.endBehavior == MotionConstraints.EndBehavior.OVERSHOOT) {
                    //we are going to overshoot so we come to a stop and then reverse back
                    //first stop
                    stoppingTimes = getDeltaVTimes(start.v, constraints);
                    profile.appendControl(-constraints.maxJ, stoppingTimes[0]);
                    profile.appendControl(0, stoppingTimes[1]);
                    profile.appendControl(constraints.maxJ, stoppingTimes[2]);
                    //now go back to the goal
                    profile.appendProfile(generateProfile(profile.end(), goal, constraints));
                    return profile;
                }
                else if(constraints.endBehavior == MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_A) {
                    // well if we are already slamming on the brakes then we might as well do infinite jerk
                    //find out how much acceleration it will take
                    double stoppingA = (Math.pow(goal.maxAbsV, 2) - Math.pow(profile.end().v, 2)) / (2 * (goal.pos - profile.end().x));
                    double stoppingTime = (goal.maxAbsV - profile.end().v) / stoppingA;
                    profile.appendInfiniteJerkControl(stoppingA, stoppingTime);
                    return profile;
                }
                else if(constraints.endBehavior == MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_V) {
                    //we will just work with what we got - we will slow down as much as possible
                    profile.appendControl(-constraints.maxJ, stoppingTimes[0]);
                    profile.appendControl(0, stoppingTimes[1]);
                    profile.appendControl(constraints.maxJ, stoppingTimes[2]);
                    double timeAtGoal = profile.timeAtPos(goal.pos);
                    return profile.trimAfter(timeAtGoal);
                }
            }
        }

        //ok cool we should be able to do it now

        double dX = goal.pos - profile.end().x;
        double j = constraints.maxJ;

        //segments 1-3
        double [] timesToAccel = getDeltaVTimes(constraints.maxV - profile.end().v, constraints);
        //segments 5-7
        double [] timesToDecel = getDeltaVTimes(goal.maxAbsV - constraints.maxV, constraints);

        //figure out how much we are going to have to coast (segment 4)
        MotionProfile accelProfile = new MotionProfile(new MotionState (0,0,0,0,0), constraints);
        accelProfile.appendControl(j, timesToAccel[0]);
        accelProfile.appendControl(0, timesToAccel[1]);
        accelProfile.appendControl(-j, timesToAccel[2] + timesToDecel[0]);
        accelProfile.appendControl(0, timesToDecel[1]);
        accelProfile.appendControl(j, timesToDecel[2]);
        double dxWithoutCoast = accelProfile.end().x;

        //ok cool we can figure out how far to coast, but this only works if we have enough time to get up to max v
        double dxCoast = dX - dxWithoutCoast;
        double timeCoast = Math.max(0, dxCoast / constraints.maxV);

        //now we need to solve the case where we can not get up to max v

        double maxV = constraints.maxV;
        double epsilon = 1E-10;
        double maxVmax = constraints.maxV;
        double maxVmin = 0;
        if (dxCoast < 0) {

            //yeah its a hack but its so much easier than the alternative, and on the plus side it converges fairly fast and could definitely be optimized
            //todo optimise this
            int iterations = 0;
            while (Math.abs(dxCoast) > epsilon && iterations < 1000) {
                iterations ++;
                if (dxCoast < 0) {
                    maxVmax = maxV;
                    maxV -= (maxV - maxVmin)/2;
                } else {
                    maxVmin = maxV;
                    maxV += (maxVmax - maxV)/2;
                }
                //this could probably be done the correct way, but this does not slow it down thaaaat much and improves readability...
                accelProfile = new MotionProfile(new MotionState(0, 0, 0, 0, 0), constraints);
                timesToAccel = getDeltaVTimes(maxV - profile.end().v, constraints);
                timesToDecel = getDeltaVTimes(goal.maxAbsV - maxV, constraints);
                accelProfile.appendControl(j, timesToAccel[0]);
                accelProfile.appendControl(0, timesToAccel[1]);
                accelProfile.appendControl(-j, timesToAccel[2] + timesToDecel[0]);
                accelProfile.appendControl(0, timesToDecel[1]);
                accelProfile.appendControl(j, timesToDecel[2]);
                dxWithoutCoast = accelProfile.end().x;
                dxCoast = dX - dxWithoutCoast;
            }
        }

        //neat now lets do it
        profile.appendControl(j, timesToAccel[0]);
        profile.appendControl(0, timesToAccel[1]);
        profile.appendControl(-j, timesToAccel[2]);
        profile.appendControl(0, timeCoast);
        profile.appendControl(-j, timesToDecel[0]);
        profile.appendControl(0, timesToDecel[1]);
        profile.appendControl(j, timesToDecel[2]);

        profile.removeEmptySegments();

        return profile;
    }

    public static MotionProfile generateStoppingProfile(MotionState start, MotionConstraints constraints) {
        double dv = -start.v;

        if (dv < 0) {
            MotionProfile profile = generateStoppingProfile(start.flipped(), constraints);
            SuperArrayList<MotionSegment> segments = profile.segments();
            //then flip the result
            SuperArrayList<MotionSegment> flipped = new SuperArrayList<>();
            for (MotionSegment seg: segments) {
                flipped.add(new MotionSegment(seg.start().flipped(), seg.dt()));
            }
            return new MotionProfile (flipped, constraints);
        }

        MotionProfile profile = new MotionProfile(start, constraints);

        if (start.a < 0) {
            double stoppingTime = Math.abs(start.a / constraints.maxJ);
            profile.appendControl(constraints.maxJ, stoppingTime);
            start = profile.end();
            dv = -start.v;
        }

        double aMax = Math.min(constraints.maxA, Math.sqrt((start.a * start.a / 2) + (dv * constraints.maxJ)));


        if (aMax > start.a) {
            double accelTime = (aMax - start.a) / constraints.maxJ;
            profile.appendControl(constraints.maxJ, accelTime);
            start = profile.end();
            dv = -start.v;
        }

        double dvDecel = (start.a * start.a) / (2.0 * constraints.maxJ);

        double dvCruise = Math.max(0.0, dv - dvDecel);


        if (dvCruise > 0.0) {
            double cruiseTime = dvCruise / start.a;
            profile.appendControl(0, cruiseTime);
            start = profile.end();
        }

        double decelTime = start.a / constraints.maxJ;
        profile.appendControl(-constraints.maxJ, decelTime);

        profile.removeEmptySegments();

        return profile;
    }

    //get the duration of the three segments needed to achieve a delta v
    private static double[] getDeltaVTimes(double dv, MotionConstraints constraints) {
        dv = Math.abs(dv / 2.0); //it should be symmetrical, so we will only deal with one half of it
        double jerkTime = constraints.maxA / constraints.maxJ; //time to max acceleration
        double maxJerkTime = Math.sqrt((2.0 * dv) / constraints.maxJ); //time to delta v
        jerkTime = Math.min(jerkTime, maxJerkTime);
        double dvInJerk = .5 * constraints.maxJ * jerkTime * jerkTime;
        double maxAccelReached = jerkTime * constraints.maxJ;
        dv = dv - dvInJerk; //now we only have to coast at max accel to finish off dv
        double coastTime = dv / maxAccelReached; //cool, now we just jerk for jerkTime, then coast for 2*coastTime, then reverse jerk for jerkTime
        return new double[] {jerkTime, coastTime * 2.0, jerkTime};
    }

    //generate a profile that requires negative delta x
    private static MotionProfile generateFlippedProfile(MotionState start, MotionGoal goal, MotionConstraints constraints) {
        //first flip the start and goal
        MotionProfile profile = generateProfile(start.flipped(), goal.flipped(), constraints);
        SuperArrayList<MotionSegment> segments = profile.segments();
        //then flip the result
        SuperArrayList<MotionSegment> flipped = new SuperArrayList<>();
        for (MotionSegment seg: segments) {
            flipped.add(new MotionSegment(seg.start().flipped(), seg.dt()));
        }
        return new MotionProfile (flipped, constraints);
    }

}
