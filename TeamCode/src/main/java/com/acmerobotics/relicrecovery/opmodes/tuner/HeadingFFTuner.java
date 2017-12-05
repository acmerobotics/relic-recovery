package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.drive.DriveConstants;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PointTurn;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import java.util.Arrays;

/**
 * @author Ryan
 */

@TeleOp(name = "Heading FF Tuner")
public class HeadingFFTuner extends LinearOpMode {
    public static final double LOWER_BOUND = 0;
    public static final double UPPER_BOUND = 0.1;
    public static final double TURN_ANGLE = Math.PI / 2;

    private RobotDashboard dashboard;
    private Looper looper;
    private MecanumDrive drive;
    private volatile double value;

    @Override
    public void runOpMode() {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());
        drive = new MecanumDrive(hardwareMap, dashboard.getTelemetry());

        looper = new Looper(20);
        drive.registerLoops(looper);
        looper.addLoop(((timestamp, dt) -> {
            telemetry.addData("value", value);
            telemetry.update();
        }));
        looper.start();

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
        DriveConstants.HEADING_COEFFS.a = coefficient;

        // reset heading + pose
        drive.setEstimatedPose(new Pose2d(0, 0, 0));
        drive.setHeading(0);

        // turn
        Path path = new Path(Arrays.asList(
                new PointTurn(new Pose2d(0, 0, 0), TURN_ANGLE)
        ));
        drive.followPath(path);

        while (opModeIsActive() && drive.isFollowingPath()) {
            sleep(10);
        }

        sleep(1000);

        double headingError = drive.getHeading() - TURN_ANGLE;

        return headingError;
    }
}
