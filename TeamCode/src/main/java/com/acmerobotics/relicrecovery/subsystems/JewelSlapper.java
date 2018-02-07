package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class JewelSlapper extends Subsystem {
    public static double DEPLOYER_UP_POSITION = 0;
    public static double DEPLOYER_DOWN_POSITION = 0.63;

    public static double SLAPPER_LEFT_POSITION = 0;
    public static double SLAPPER_CENTER_POSITION = 0.59;
    public static double SLAPPER_RIGHT_POSITION = 1;
    public static double SLAPPER_STOW_POSITION = 0.87;

    private Telemetry telemetry;

    public enum SlapperPosition {
        LEFT,
        CENTER,
        RIGHT,
        STOW
    }

    private Servo jewelDeployer, jewelSlapper;

    private SlapperPosition slapperPosition;
    private boolean deployed;

    public JewelSlapper(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        jewelDeployer = map.servo.get("jewelDeployer");
        jewelSlapper = map.servo.get("jewelSlapper");

        stowArmAndSlapper();
    }

    public void deployArmAndSlapper() {
        setSlapperPosition(SlapperPosition.CENTER);
        deployed = true;
    }

    public void stowArmAndSlapper() {
        setSlapperPosition(SlapperPosition.STOW);
        deployed = false;
    }

    public SlapperPosition getSlapperPosition() {
        return slapperPosition;
    }

    public void setSlapperPosition(SlapperPosition slapperPosition) {
        this.slapperPosition = slapperPosition;
    }

    @Override
    public void update() {
        telemetry.addData("jewelSlapperDeployed", deployed);
        telemetry.addData("jewelSlapperPosition", slapperPosition);

        if (deployed) {
            jewelDeployer.setPosition(DEPLOYER_DOWN_POSITION);
        } else {
            jewelDeployer.setPosition(DEPLOYER_UP_POSITION);
        }

        switch (slapperPosition) {
            case LEFT:
                jewelSlapper.setPosition(SLAPPER_LEFT_POSITION);
                break;
            case CENTER:
                jewelSlapper.setPosition(SLAPPER_CENTER_POSITION);
                break;
            case RIGHT:
                jewelSlapper.setPosition(SLAPPER_RIGHT_POSITION);
                break;
            case STOW:
                jewelSlapper.setPosition(SLAPPER_STOW_POSITION);
                break;
        }
    }
}
