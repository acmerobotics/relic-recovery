package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Angle;
import com.acmerobotics.relicrecovery.localization.Vector2d;

/**
 * @author Ryan
 */

public class LinearPathExecutor {
    public static final double MOVEMENT_SPEED = 0.5;

    private MecanumDrive drive;

    public LinearPathExecutor(MecanumDrive drive) {
        this.drive = drive;
    }

    public void execute(LinearPath path, double speed, double initialHeading) {
        double heading = initialHeading;
        for (LinearPath.Segment segment : path.getSegments()) {
            double deltaX = segment.end().x() - segment.start().x();
            double deltaY = segment.end().y() - segment.start().y();
            double newHeading = Angle.norm(Math.atan2(deltaY, deltaX) - Math.PI / 2);
            double turnAngle = Angle.norm(newHeading - heading);
            if (turnAngle > Vector2d.EPSILON) {
                drive.turnSync(turnAngle, null);
            }
            double length = segment.length();
            if (length > Vector2d.EPSILON) {
                drive.move(length, speed, null);
            }
            heading = newHeading;
        }
    }
}
