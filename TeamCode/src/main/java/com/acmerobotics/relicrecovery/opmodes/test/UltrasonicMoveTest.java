package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
                .beginComposite()
                .lineTo(new Vector2d(12, -36))
                .addMarker("ultrasonic")
                .lineTo(new Vector2d(12, -56))
                .closeComposite()
                .build();
        robot.drive.setEstimatedPose(trajectory.start());
        robot.drive.followTrajectory(trajectory);
        robot.drive.waitForMarker("ultrasonic");
        ultrasonicLocalizer.enableUltrasonicFeedback();
        robot.drive.waitForTrajectoryFollower();
        ultrasonicLocalizer.disableUltrasonicFeedback();
    }
}
