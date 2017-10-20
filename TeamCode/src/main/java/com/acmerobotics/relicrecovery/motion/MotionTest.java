package com.acmerobotics.relicrecovery.motion;

import java.io.FileWriter;
import java.io.IOException;

/**
 * Created by kelly on 10/19/2017.
 *
 */

public class MotionTest {

    public static void main(String[] args) {
        doATest("basic", 16, 0,0);
        doATest("violateA", 5, 5, 0, MotionConstraints.END_BEHAVIOR.VIOLATE_MAX_ABS_A);
        doATest("violateV", 5, 5, 0, MotionConstraints.END_BEHAVIOR.VIOLATE_MAX_ABS_V);
        doATest("overshoot", 5, 5, 0, MotionConstraints.END_BEHAVIOR.OVERSHOOT);
        doATest("reversed", -10, 0,0);
        doATest("short", 2, 0,0);

    }

    public static void doATest(String name, double target, double startVel, double endVel, MotionConstraints.END_BEHAVIOR behavior) {
        MotionState state = new MotionState(0, startVel, 0, 0, 0);
        MotionGoal goal = new MotionGoal(target, endVel);
        MotionConstraints constraints = new MotionConstraints();
        constraints.endBehavior = behavior;
        constraints.maxA = 100;
        constraints.maxJ = 1;
        constraints.maxV = 100;
        MotionProfile profile = MotionProfileGenerator.generateProfile(state, goal, constraints);
        writeProfile(profile, name);
    }

    public static void doATest(String name, double goal, double startVel, double endVel) {
        doATest(name, goal, startVel, endVel, MotionConstraints.END_BEHAVIOR.OVERSHOOT);
    }

    public static void writeProfile(MotionProfile profile, String name) {
        try {
            FileWriter writer = new FileWriter("./out/" + name + ".csv");
            writer.write("x, v, a, j\n");
            for(double t = 0; t < profile.end().t; t += .01) {
                MotionState state = profile.get(t);
                writer.write(String.format("%f, %f, %f, %f\n", state.x, state.v, state.a, state.j));
            }
            writer.close();
        } catch (IOException e) {
            e.printStackTrace();
        }
    }
}
