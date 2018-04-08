package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@Autonomous
public class UltrasonicMoveTest extends AutoOpMode {
    private UltrasonicLocalizer ultrasonicLocalizer;

    @Override
    protected void setup() {
        ultrasonicLocalizer = new UltrasonicLocalizer(robot.drive);
        robot.addListener(() -> {
            robot.dashboard.getTelemetry().addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
            telemetry.addData("ultrasonicDistance", ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));
        });
        robot.drive.setLocalizer(ultrasonicLocalizer);
        robot.drive.extendUltrasonicSwivel();
    }

    @Override
    protected void run() {
        Trajectory trajectory = new TrajectoryBuilder(new Pose2d(12, 0, Math.PI / 2))
                .lineTo(new Vector2d(12, -36))
                .waitFor(0.25)
                .build();
        robot.drive.setEstimatedPose(trajectory.start());
        robot.drive.followTrajectory(trajectory);
        robot.drive.waitForTrajectoryFollower();

        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.waitOneFullCycle();
        ultrasonicLocalizer.disableUltrasonicFeedback();

        RobotLog.i("Distance: " + ultrasonicLocalizer.getUltrasonicDistance(DistanceUnit.INCH));

        Trajectory trajectory2 = new TrajectoryBuilder(new Pose2d(robot.drive.getEstimatedPosition(), Math.PI / 2))
                .waitFor(10.0)
                .lineTo(new Vector2d(12, -56))
                .waitFor(0.5)
                .build();
        robot.drive.followTrajectory(trajectory2);
        robot.drive.waitForTrajectoryFollower();
    }
}
