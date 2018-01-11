package com.acmerobotics.relicrecovery.subsystems;

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

    private Servo jewelDeployer, jewelSlapper;

    private Position position;
    private boolean deployed;

    public JewelSlapper(HardwareMap map) {
        jewelDeployer = map.servo.get("jewelDeployer");
        jewelSlapper = map.servo.get("jewelSlapper");

        setPosition(Position.LEFT);
    }

    public void deploy() {
        if (!deployed) {
            jewelDeployer.setPosition(1);
            deployed = true;
        }
    }

    public void undeploy() {
        if (deployed) {
            jewelDeployer.setPosition(0);
            deployed = false;
        }
    }

    public Position getPosition() {
        return position;
    }

    public void setPosition(Position position) {
        if (this.position != position) {
            jewelSlapper.setPosition(position.getServoPosition());
            this.position = position;
        }
    }
}
