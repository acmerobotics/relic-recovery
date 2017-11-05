package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;

import org.junit.Test;
import org.mockito.InOrder;

import java.util.Arrays;

import static org.mockito.Mockito.inOrder;
import static org.mockito.Mockito.mock;

/**
 * @author Ryan
 */

public class LinearPathExecutorTest {

    @Test
    public void testBasicPathExecution() {
        MecanumDrive drive = mock(MecanumDrive.class);

        LinearPath path = new LinearPath(Arrays.asList(
                new Pose2d(0, 0),
                new Pose2d(2, 0),
                new Pose2d(2, 2)
        ));
        LinearPathExecutor executor = new LinearPathExecutor(drive);
        executor.execute(path, 0.5, 0);

        InOrder inOrder = inOrder(drive);
        inOrder.verify(drive).move(2, 0.5);
        inOrder.verify(drive).turn(Math.PI / 2);
        inOrder.verify(drive).move(2, 0.5);
    }

}
