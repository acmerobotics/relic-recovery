package com.acmerobotics.relicrecovery.motion;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class MotionTest {

    public static void main(String[] args) {

        MotionConstraints constraints = new MotionConstraints(5, 5, 5, MotionConstraints.EndBehavior.OVERSHOOT);

        MotionState start = new MotionState (0, 3, 2, 0, 0);
        MotionProfile profile = MotionProfileGenerator.generateStoppingProfile(start, constraints);

        writeProfile(profile, "stop");

    }

    public static void doATest(String name, double target, double startVel, double endVel, MotionConstraints.EndBehavior behavior) {
        MotionState state = new MotionState(0, startVel, 0, 0, 0);
        MotionGoal goal = new MotionGoal(target, endVel);
        MotionConstraints constraints = new MotionConstraints(2, 1, 1, behavior);
        MotionProfile profile = MotionProfileGenerator.generateProfile(state, goal, constraints);
        writeProfile(profile, name);
    }

    public static void doATest(String name, double goal, double startVel, double endVel) {
        doATest(name, goal, startVel, endVel, MotionConstraints.EndBehavior.OVERSHOOT);
    }

    public static void writeProfile(MotionProfile profile, String name) {
        try {
            new File("./out").mkdirs();
            FileWriter writer = new FileWriter("./out/" + name + ".csv");
            writer.write("t, x, v, a, j\n");
            for(double t = 0; t < profile.end().t; t += .01) {
                MotionState state = profile.get(t);
                writer.write(String.format("%f, %f, %f, %f, %f\n", t, state.x, state.v, state.a, state.j));
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    // TODO: this too
    // also, consider ways to make this into a unit test

//    public static void writeSegment(PathSegment seg, String name) {
//        try {
//            FileWriter writer = new FileWriter("./out/paths/" + name + ".csv");
//            writer.write("t, x, y, heading\n");
//            for (double t = 0; t < seg.headingProfile().end().t; t += .01) {
//                Pose2d pose = seg.getPose(t);
//                writer.write(String.format("%f, %f, %f, %f\n", t, pose.x(), pose.y(), pose.heading()));
//            }
//            writer.close();
//        } catch (IOException e) {
//            e.printStackTrace();
//        }
//    }
}
