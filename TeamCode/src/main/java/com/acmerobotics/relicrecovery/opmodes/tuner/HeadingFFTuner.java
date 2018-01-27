package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

@TeleOp(name = "Heading FF Tuner")
public class HeadingFFTuner extends LinearOpMode {
    public static final double LOWER_BOUND = 0;
    public static final double UPPER_BOUND = 0.075;
    public static final double TURN_ANGLE = Math.PI / 2;

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
        MecanumDrive.HEADING_PID.a = coefficient;

        // reset heading + pose
        robot.drive.setEstimatedPose(new Pose2d(0, 0, 0));

        // turn
        Path path = new Path(Arrays.asList(
                new PointTurn(new Pose2d(0, 0, 0), TURN_ANGLE)
        ));
        robot.drive.followPath(path);

        telemetry.addData("headingKa", value);
        telemetry.update();

        double errorSum = 0;
        int numErrors = 0;

        while (opModeIsActive() && robot.drive.isFollowingPath()) {
            errorSum += robot.drive.getPathFollower().getHeadingError();
            numErrors++;
            sleep(5);
        }

        sleep(1000);

        return errorSum / numErrors;
    }
}
