package com.acmerobotics.relicrecovery.opmodes.auto;

import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.BalancingStone;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.acmerobotics.relicrecovery.opmodes.AutoPaths;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;

@Autonomous
public class NormalAuto extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        RelicRecoveryVuMark vuMark = scoreJewelAndReadPictograph();

        BalancingStone balancingStone = robot.config.getBalancingStone();
        Path path = AutoPaths.makeNormalPathToCryptobox(balancingStone, vuMark);
        Pose2d initialPose = path.getPose(0);
        robot.drive.setEstimatedPose(initialPose);

        followPathSync(path);

        robot.dumpBed.dump();
        sleep(1000);
        robot.drive.setVelocity(new Vector2d(0.2, 0), 0);
        sleep(750);
        robot.drive.stop();
        sleep(1000);
        robot.dumpBed.retract();
    }
}
