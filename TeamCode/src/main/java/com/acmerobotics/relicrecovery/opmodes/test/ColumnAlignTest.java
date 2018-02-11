package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ColumnAlignTest extends AutoOpMode {
    @Override
    protected void setup() {
        robot.drive.extendSideSwivel();
    }

    @Override
    protected void run() {
        robot.drive.alignWithColumn();
        robot.drive.waitForColumnAlign();

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
