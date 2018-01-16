package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/9/18.
 */

public class JewelSlapper {
    public enum Position {
        LEFT(0),
        CENTER(0.5),
        RIGHT(1);

        private double servoPos;

        Position(double servoPos) {
            this.servoPos = servoPos;
        }

        public double getServoPosition() {
            return servoPos;
        }
    }

    private PriorityScheduler scheduler;

    private Servo jewelDeployer, jewelSlapper;

    private Position position;
    private boolean deployed;

    public JewelSlapper(HardwareMap map, PriorityScheduler scheduler) {
        this.scheduler = scheduler;

        jewelDeployer = map.servo.get("jewelDeployer");
        jewelSlapper = map.servo.get("jewelSlapper");

        setPosition(Position.LEFT);
    }

    public void deploy() {
        if (!deployed) {
            scheduler.add(() -> jewelDeployer.setPosition(0), "jewel: deploy set pos", PriorityScheduler.HIGH_PRIORITY);
            deployed = true;
        }
    }

    public void undeploy() {
        if (deployed) {
            scheduler.add(() -> jewelDeployer.setPosition(1), "jewel: deploy set pos", PriorityScheduler.HIGH_PRIORITY);
            deployed = false;
        }
    }

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        if (this.position == position) {
            scheduler.add(() -> jewelSlapper.setPosition(position.getServoPosition()), "jewel: slapper set pos", PriorityScheduler.HIGH_PRIORITY);
            this.position = position;
        }
    }
}
