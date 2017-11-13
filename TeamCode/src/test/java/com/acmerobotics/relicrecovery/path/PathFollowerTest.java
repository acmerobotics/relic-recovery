package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;

import org.junit.Test;

import static org.mockito.ArgumentMatchers.anyDouble;
import static org.mockito.Mockito.mock;
import static org.mockito.Mockito.when;
import static org.mockito.Mockito.withSettings;

/**
 * @author Ryan
 */

public class PathFollowerTest {

    @Test
    public void testPathFollower() {
        Path path = mock(Path.class);

        when(path.getPose(anyDouble())).thenReturn(new Pose2d(0, 0, Math.PI / 4));
        when(path.getPoseVelocity(anyDouble())).thenReturn(new Pose2d(0, 0, 0));
        when(path.getPoseAcceleration(anyDouble())).thenReturn(new Pose2d(0, 0, 0));
        when(path.duration()).thenReturn(100.0);

        Pose2d robotPose = new Pose2d(-1, 1, Math.PI / 4);

        MecanumDrive drive = mock(MecanumDrive.class, withSettings().verboseLogging());

        PIDFCoefficients emptyPIDF = new PIDFCoefficients(1, 0, 0, 0, 0);
        PathFollower follower = new PathFollower(drive, null, emptyPIDF, emptyPIDF, emptyPIDF);

        follower.follow(path);
        follower.update(robotPose, System.currentTimeMillis());
    }

}
