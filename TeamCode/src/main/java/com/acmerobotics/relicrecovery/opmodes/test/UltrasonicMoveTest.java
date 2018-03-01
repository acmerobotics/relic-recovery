package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path2.PathBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous
public class UltrasonicMoveTest extends AutoOpMode {
    private UltrasonicLocalizer ultrasonicLocalizer;

    @Override
    protected void setup() {
        ultrasonicLocalizer = new UltrasonicLocalizer(robot.drive);
        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.addListener(() -> {
            robot.dashboard.getTelemetry().addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
        });
        robot.drive.setLocalizer(ultrasonicLocalizer);
        robot.drive.setEstimatedPose(new Pose2d(12, -24, Math.PI / 2));
    }

    @Override
    protected void run() {
        robot.drive.followPath(new PathBuilder(new Pose2d(12, -24, Math.PI / 2))
                .lineTo(new Vector2d(12, -54))
                .build());
        robot.drive.waitForPathFollower();

        sleep(500);

        robot.dumpBed.dump();

        sleep(1000);

        robot.drive.setVelocity(new Vector2d(0.2, 0), 0);

        sleep(750);

        robot.drive.stop();

        sleep(500);

        robot.dumpBed.retract();

        sleep(1000);
    }
}
