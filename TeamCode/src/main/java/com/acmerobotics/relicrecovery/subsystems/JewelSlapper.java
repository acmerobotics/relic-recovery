package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class JewelSlapper extends Subsystem {
    public static final double DEPLOYER_UP_POSITION = 0.07;
    public static final double DEPLOYER_DOWN_POSITION = 0.63;

    private Telemetry telemetry;

    public enum Position {
        LEFT(0.4),
        CENTER(0.59),
        RIGHT(0.73),
        STOW(0.87);

        private double servoPos;

        Position(double servoPos) {
            this.servoPos = servoPos;
        }

        public double getServoPosition() {
            return servoPos;
        }
    }

    private Servo jewelDeployer, jewelSlapper;

    private Position position = Position.STOW;
    private boolean deployed;

    public JewelSlapper(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        jewelDeployer = map.servo.get("jewelDeployer");
        jewelSlapper = map.servo.get("jewelSlapper");

        deployed = true;
        stowArmAndSlapper();
    }

    public void deployArmAndSlapper() {
        if (!deployed) {
            jewelDeployer.setPosition(DEPLOYER_DOWN_POSITION);
            setSlapperPosition(Position.CENTER);
            deployed = true;
        }
    }

    public void stowArmAndSlapper() {
        if (deployed) {
            jewelDeployer.setPosition(DEPLOYER_UP_POSITION);
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

    @Override
    public void update() {
        telemetry.addData("jewelSlapperDeployed", deployed);
        telemetry.addData("jewelSlapperPosition", position);
    }
}
