package com.acmerobotics.relicrecovery.motion;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;

import java.io.FileWriter;
import java.io.IOException;

/**
 * @author kellyrm
 *
 */

public class MotionTest {

    public static void main(String[] args) {
        /*doATest("basic", 16, 0,0);
        doATest("violateA", 5, 5, 0, MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_A);
        doATest("violateV", 5, 5, 0, MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_V);
        doATest("overshoot", 5, 5, 0, MotionConstraints.EndBehavior.OVERSHOOT);
        doATest("reversed", -10, 0,0);
        doATest("short", 2.3, 0,0);*/
        MotionState state = new MotionState(0, 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(2.3, 0);
        MotionConstraints constraints = new MotionConstraints(2, 1, 1, MotionConstraints.EndBehavior.OVERSHOOT);
        MotionProfile profile = MotionProfileGenerator.generateProfile(state, goal, constraints);
        profile = profile.stretch(20);
        writeProfile(profile, "shortNormal");


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
}
