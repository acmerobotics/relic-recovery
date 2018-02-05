package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous
public class ColumnAlignTest extends AutoOpMode {
    @Override
    protected void setup() {

    }

    @Override
    protected void run() {
        alignWithColumnSync();
    }
}
