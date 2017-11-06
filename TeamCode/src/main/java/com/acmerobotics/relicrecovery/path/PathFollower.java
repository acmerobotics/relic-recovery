package com.acmerobotics.relicrecovery.path;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Angle;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.localization.Vector2d;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

/**
 * @author Ryan
 */

@Config
public class PathFollower {
    public static PIDCoefficients HEADING_COEFFS = new PIDCoefficients();
    public static PIDCoefficients AXIAL_COEFFS = new PIDCoefficients();
    public static PIDCoefficients LATERAL_COEFFS = new PIDCoefficients();

    private MecanumDrive drive;
    private PIDController headingController, axialController, lateralController;
    private Path path;
    private PathSegment segment;
    private int segmentIndex;

    public PathFollower(MecanumDrive drive) {
        this.drive = drive;

        headingController = new PIDController(HEADING_COEFFS);
        headingController.setInputBounds(-Math.PI / 2, Math.PI / 2);
        headingController.setOutputBounds(-1, 1);

        axialController = new PIDController(AXIAL_COEFFS);
        axialController.setOutputBounds(-1, 1);

        lateralController = new PIDController(LATERAL_COEFFS);
        lateralController.setOutputBounds(-1, 1);
    }

    public void follow(Path path) {
        this.path = path;
        this.segmentIndex = 0;
        this.segment = path.getSegment(0);
    }

    public void update(Pose2d pose) {
        Pose2d
    }

    public void execute(Path path, double speed, double initialHeading) {
        double heading = initialHeading;
        for (LinearSegment segment : path.getSegments()) {
            double deltaX = segment.end().x() - segment.start().x();
            double deltaY = segment.end().y() - segment.start().y();
            double newHeading = Angle.norm(Math.atan2(deltaY, deltaX) - Math.PI / 2);
            double turnAngle = Angle.norm(newHeading - heading);
            if (turnAngle > Vector2d.EPSILON) {
                drive.turn(turnAngle);
            }
            double length = segment.length();
            if (length > Vector2d.EPSILON) {
                drive.move(length, speed);
            }
            heading = newHeading;
        }
    }
}
