package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;

/**
 * Created by ryanbrott on 10/28/17.
 */

@Config
public class DriveConstants {
    public static MotionConstraints DRIVE_MTION_CONSTRAINTS = new MotionConstraints(0, 0, 0, MotionConstraints.EndBehavior.OVERSHOOT);
}
