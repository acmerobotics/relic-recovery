package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.localization.Pose2d;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import java.util.Arrays;

/**
 * Created by ryanbrott on 11/7/17.
 */

@Autonomous
public class LineTest extends LinearOpMode {
    private RobotDashboard dashboard;
    private MecanumDrive drive;
    private Looper looper;

    @Override
    public void runOpMode() throws InterruptedException {
        Path path = Path.createFromPoses(Arrays.asList(
                new Pose2d(0, 0),
                new Pose2d(36, 0)
        ));

        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        looper = new Looper(20);

        drive = new MecanumDrive(hardwareMap);
        drive.registerLoops(looper);

        waitForStart();

        looper.start();

        drive.followPath(path);

        while (opModeIsActive()) {
            telemetry.update();

            sleep(20);
        }

        looper.terminate();
    }
}
