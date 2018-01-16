package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/9/18.
 */

public class JewelSlapper {
    public enum Position {
        LEFT(1),
        CENTER(0.75),
        RIGHT(0.43),
        STOW(0);

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

        stowArmAndSlapper();
    }

    public void deployArmAndSlapper() {
        if (!deployed) {
            jewelDeployer.setPosition(0.77);
            setSlapperPosition(Position.CENTER);
            deployed = true;
        }
    }

    public void stowArmAndSlapper() {
        if (deployed) {
            jewelDeployer.setPosition(0.01);
            setSlapperPosition(Position.STOW);
            deployed = false;
        }
    }

    public Position getPosition() {
        return position;
    }

    public void setSlapperPosition(Position position) {
        if (this.position != position) {
            jewelSlapper.setPosition(position.getServoPosition());
            this.position = position;
        }
    }
}
