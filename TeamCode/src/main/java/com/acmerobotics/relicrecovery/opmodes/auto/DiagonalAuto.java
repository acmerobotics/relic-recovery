package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous
public class DiagonalAuto extends AutoOpMode {
    @Override
    protected void setup() {
        Pose2d initialPose = AutoPaths.getAdjustedBalancingStonePose(robot.config.getBalancingStone());
        robot.drive.setEstimatedPose(initialPose);
    }

    @Override
    protected void run() {
        // jewel logic here
        RelicRecoveryVuMark vuMark = vuMarkTracker.getVuMark();
        JewelPosition jewelPosition = jewelTracker.getJewelPosition();
        jewelTracker.disable();

        robot.jewelSlapper.lowerArmAndSlapper();

        robot.sleep(0.75);

        boolean removeLeft = robot.config.getAllianceColor() == jewelPosition.rightColor();

        if (removeLeft && robot.config.getAllianceColor() == AllianceColor.BLUE) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.LEFT);
            robot.sleep(0.5);
            robot.jewelSlapper.stowArmAndSlapper();
        } else if (!removeLeft && robot.config.getAllianceColor() == AllianceColor.RED) {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.RIGHT);
            robot.sleep(0.5);
            robot.jewelSlapper.stowArmAndSlapper();
        } else {
            robot.jewelSlapper.setSlapperPosition(JewelSlapper.SlapperPosition.PARALLEL);
            robot.sleep(0.5);
        }

        Path path = AutoPaths.makeDiagonalPathToCryptobox(robot.config.getBalancingStone(), vuMark);
        robot.drive.followPath(path);
        robot.sleep(0.5);
        robot.jewelSlapper.stowArmAndSlapper();
        robot.drive.waitForPathFollower();

        robot.dumpBed.dump();
        sleep(1000);
        robot.drive.setVelocity(new Vector2d(0.2, 0), 0);
        sleep(750);
        robot.drive.stop();
        sleep(1000);
        robot.dumpBed.retract();
        sleep(500);
    }
}