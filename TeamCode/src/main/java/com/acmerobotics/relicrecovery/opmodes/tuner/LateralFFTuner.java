package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.path.LineSegment;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

/**
 * @author Ryan
 */

@TeleOp(name = "Lateral FF Tuner")
public class LateralFFTuner extends LinearOpMode {
    public static final double LOWER_BOUND = 0;
    public static final double UPPER_BOUND = 0.001;
    public static final double DISTANCE = 48;

    private RobotDashboard dashboard;
    private MecanumDrive drive;
    private volatile double value;

    @Override
    public void runOpMode() {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, telemetry);

        waitForStart();

        double lower = LOWER_BOUND, upper = UPPER_BOUND;

        while (opModeIsActive()) {
            value = (lower + upper) / 2;

            int numErrors = 3;
            double errorSum = 0;
            for (int i = 0; i < numErrors; i++) {
                errorSum += testFeedforwardCoefficient(value);
            }
            double error = errorSum / numErrors;
            if (error > 0) {
                upper = value;
            } else {
                lower = value;
            }
        }
    }

    private double testFeedforwardCoefficient(double coefficient) {
        MecanumDrive.LATERAL_PID.a = coefficient;

        // reset heading + pose
        drive.setEstimatedPose(new Pose2d(0, 0, drive.getHeading()));

        Path forward = new Path(Arrays.asList(
                new LineSegment(new Pose2d(0, 0, 0), new Pose2d(0, DISTANCE, 0))
        ));
        Path reverse = new Path(Arrays.asList(
                new LineSegment(new Pose2d(0, DISTANCE, 0), new Pose2d(0, 0, 0))
        ));

        drive.followPath(forward);

        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.update();

            telemetry.addData("value", value);
            telemetry.update();
        }

        double axialError = drive.getEstimatedPose().y() - DISTANCE;

        drive.followPath(reverse);

        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.update();

            telemetry.addData("value", value);
            telemetry.update();
        }

        return axialError;
    }
}
