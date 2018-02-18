package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class JewelSlapper extends Subsystem {
    public static double ARM_UP_POSITION = 0.17;
    public static double ARM_HALFWAY_POSITION = 0.6;
    public static double ARM_DOWN_POSITION = 0.85;

    public static double SLAPPER_LEFT_POSITION = 0.85;
    public static double SLAPPER_PARALLEL_POSITION = 0.49;
    public static double SLAPPER_CENTER_POSITION = 0.59;
    public static double SLAPPER_RIGHT_POSITION = 0.25;
    public static double SLAPPER_STOW_POSITION = 0.09;

    private Telemetry telemetry;

    public enum ArmPosition {
        UP,
        HALFWAY,
        DOWN
    }

    public enum SlapperPosition {
        LEFT,
        CENTER,
        RIGHT,
        STOW,
        PARALLEL
    }

    private Servo jewelArm, jewelSlapper;

    private SlapperPosition slapperPosition;
    private ArmPosition armPosition;

    public JewelSlapper(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        jewelArm = map.servo.get("jewelArm");
        jewelSlapper = map.servo.get("jewelSlapper");

        stowArmAndSlapper();
    }

    public void lowerArmAndSlapper() {
        setSlapperPosition(SlapperPosition.CENTER);
        setArmPosition(ArmPosition.DOWN);
    }

    public void stowArmAndSlapper() {
        setSlapperPosition(SlapperPosition.STOW);
        setArmPosition(ArmPosition.UP);
    }

    public void setSlapperPosition(SlapperPosition slapperPosition) {
        this.slapperPosition = slapperPosition;
    }

    public void setArmPosition(ArmPosition armPosition) {
        this.armPosition = armPosition;
    }

    @Override
    public void update() {
        telemetry.addData("jewelArmPosition", armPosition);
        telemetry.addData("jewelSlapperPosition", slapperPosition);

        switch (armPosition) {
            case UP:
                jewelArm.setPosition(ARM_UP_POSITION);
                break;
            case HALFWAY:
                jewelArm.setPosition(ARM_HALFWAY_POSITION);
                break;
            case DOWN:
                jewelArm.setPosition(ARM_DOWN_POSITION);
                break;
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
            case PARALLEL:
                jewelSlapper.setPosition(SLAPPER_PARALLEL_POSITION);
                break;
        }
    }
}
