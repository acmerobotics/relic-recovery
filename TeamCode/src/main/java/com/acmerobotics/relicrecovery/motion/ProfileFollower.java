package com.acmerobotics.relicrecovery.motion;

import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * @author kellyrm
 *
 */

public class ProfileFollower {

    public class FeedforwardCoefficients {
        public double kA;
        public double kV;

        public FeedforwardCoefficients() {
            kA = kV = 0;
        }

        public FeedforwardCoefficients(double kA, double kV) {
            this.kA = kA;
            this.kV = kV;
        }
    }

    private MotionProfile profile;
    private FeedforwardCoefficients ff;
    private PIDController pid;

    public ProfileFollower (MotionProfile profile, PIDCoefficients pidC, FeedforwardCoefficients ff) {
        this.profile = profile;
        this.ff = ff;
        this.pid = new PIDController(pidC);
    }

    public double update(MotionState now) {
        MotionState target = profile.get(now.t);
        double error = target.x - now.x;
        double pidUpdate = pid.update(error, now.t);
        return pidUpdate + (ff.kA * target.a) + (ff.kV * target.v);
    }

}
