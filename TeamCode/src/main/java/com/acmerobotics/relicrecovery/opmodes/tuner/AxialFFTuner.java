package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path.LineSegment;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "Axial FF Tuner")
public class AxialFFTuner extends LinearOpMode {
    public static final double LOWER_BOUND = 0;
    public static final double UPPER_BOUND = 0.01;
    public static final double DISTANCE = 60;

    private Robot robot;
    private volatile double value;

    @Override
    public void runOpMode() {
        robot = new Robot(this);

        robot.drive.enablePositionEstimation();

        robot.start();

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
        MecanumDrive.AXIAL_PID.a = coefficient;

        // reset heading + pose
        robot.drive.setEstimatedPose(new Pose2d(0, 0, 0));

        Path forward = new Path(Arrays.asList(
                new LineSegment(new Pose2d(0, 0, 0), new Pose2d(DISTANCE, 0, 0))
        ));
        Path reverse = new Path(Arrays.asList(
                new LineSegment(new Pose2d(DISTANCE, 0, 0), new Pose2d(0, 0, 0))
        ));
        robot.drive.followPath(forward);

        telemetry.addData("value", value);
        telemetry.update();

        double errorSum = 0;
        int numErrors = 0;

        while (opModeIsActive() && robot.drive.isFollowingPath()) {
            errorSum += robot.drive.getPathFollower().getAxialError();
            numErrors++;
            sleep(5);
        }

        robot.drive.followPath(reverse);

        while (opModeIsActive() && robot.drive.isFollowingPath()) {
            sleep(5);
        }

        return errorSum / numErrors;
    }
}
