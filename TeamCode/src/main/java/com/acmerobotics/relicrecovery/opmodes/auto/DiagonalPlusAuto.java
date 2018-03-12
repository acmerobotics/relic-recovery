package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Trajectory;
import com.acmerobotics.relicrecovery.path.TrajectoryBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous(name = "Diagonal+ Auto")
public class DiagonalPlusAuto extends AutoOpMode {
    @Override
    protected void setup() {
        Pose2d initialPose = AutoPaths.getAdjustedBalancingStonePose(robot.config.getBalancingStone());
        robot.drive.setEstimatedPose(initialPose);
    }

    @Override
    protected void run() {
        int yMultiplier = robot.config.getAllianceColor() == AllianceColor.BLUE ? -1 : 1;

        // jewel logic here
        RelicRecoveryVuMark vuMark = vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        lowerArmAndSlapper();

        boolean removeLeft = robot.config.getAllianceColor() == jewelPosition.rightColor();

        if (removeLeft && robot.config.getAllianceColor() == AllianceColor.BLUE) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
            robot.sleep(0.75);
            raiseArmAndSlapper();
        } else if (!removeLeft && robot.config.getAllianceColor() == AllianceColor.RED) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
            robot.sleep(0.75);
            raiseArmAndSlapper();
        } else {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.PARALLEL);
            robot.sleep(0.75);
        }

        Trajectory stoneToCrypto = AutoPaths.makeDiagonalTrajectoryToCryptobox(robot.config.getBalancingStone(), vuMark);
        robot.drive.followTrajectory(stoneToCrypto);
        robot.sleep(0.5);
        raiseArmAndSlapper();
        robot.drive.waitForTrajectoryFollower();

        double pileAngle = -yMultiplier * Math.PI / 4;
        double turnAngle = Angle.norm(pileAngle - stoneToCrypto.end().heading());

        robot.dumpBed.dump();
        sleep(500);
        Trajectory cryptoToPit = new TrajectoryBuilder(stoneToCrypto.end())
                .lineTo(new Vector2d(stoneToCrypto.end().x(), yMultiplier * 12))
                .turn(turnAngle)
                .forward(12)
                .back(12)
                .turn(-yMultiplier * Math.PI / 4)
                .lineTo(new Vector2d(stoneToCrypto.end().x(), yMultiplier * 48))
                .build();
        robot.drive.followTrajectory(cryptoToPit);
        sleep(500);
        robot.dumpBed.retract();
        robot.intake.autoIntake();
        robot.drive.waitForTrajectoryFollower();
    }
}