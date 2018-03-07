package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathBuilder;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.vision.JewelPosition;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Config
@Autonomous(name = "Diagonal Auto")
public class DiagonalAuto extends AutoOpMode {
    public static RelicRecoveryVuMark VUMARK = RelicRecoveryVuMark.CENTER;

    @Override
    protected void setup() {
        Pose2d initialPose = AutoPaths.getAdjustedBalancingStonePose(robot.config.getBalancingStone());
        robot.drive.setEstimatedPose(initialPose);
    }

    @Override
    protected void run() {
        // jewel logic here
        RelicRecoveryVuMark vuMark = VUMARK; // vuMarkTracker.getVuMark();
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

        Path stoneToCrypto = AutoPaths.makeDiagonalPathToCryptobox(robot.config.getBalancingStone(), vuMark);
        robot.drive.followPath(stoneToCrypto);
        robot.sleep(0.5);
        robot.jewelSlapper.stowArmAndSlapper();
        robot.drive.waitForPathFollower();

        robot.dumpBed.dump();
        sleep(500);
        Path cryptoToPit = new PathBuilder(stoneToCrypto.end())
                .forward(6)
                .build();
        robot.drive.followPath(cryptoToPit);
        sleep(500);
        robot.dumpBed.retract();
        robot.drive.waitForPathFollower();
    }
}