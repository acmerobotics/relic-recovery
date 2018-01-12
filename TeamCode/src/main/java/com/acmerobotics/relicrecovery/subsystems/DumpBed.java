package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 1/9/18.
 */

@Config
public class DumpBed {
    public static final double LIFT_POWER_DOWN = -0.4;
    public static final double LIFT_POWER_UP = 0.8;

    public static final double LEFT_ROTATE_DOWN_POS = 0.25;
    public static final double LEFT_ROTATE_UP_POS = 0.78;

    public static final double RELEASE_ENGAGE_POS = 0.61;
    public static final double RELEASE_DISENGAGE_POS = 0;

    public enum Mode {
        NORMAL,
        MOVE_DOWN,
        MOVE_UP
    }

    private Mode mode = Mode.NORMAL;

    private DcMotor dumpLiftLeft, dumpLiftRight;
    private Servo dumpRelease, dumpRotateLeft, dumpRotateRight;
    private DigitalChannel dumpLiftBottomTouch, dumpLiftTopTouch;

    private boolean liftDumping, releaseEngaged, liftDown;

    private Telemetry telemetry;

    public DumpBed(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        dumpLiftLeft = map.dcMotor.get("dumpLiftLeft");
        dumpLiftRight = map.dcMotor.get("dumpLiftRight");

        dumpRotateLeft = map.servo.get("dumpRotateLeft");
        dumpRotateRight = map.servo.get("dumpRotateRight");
        dumpRelease = map.servo.get("dumpRelease");

        dumpLiftBottomTouch = map.digitalChannel.get("dumpLiftBottomTouch");
        dumpLiftBottomTouch.setMode(DigitalChannel.Mode.INPUT);
        dumpLiftTopTouch = map.digitalChannel.get("dumpLiftTopTouch");
        dumpLiftTopTouch.setMode(DigitalChannel.Mode.INPUT);

        engageRelease();
        setDumpRotation(0);
    }

    private void setLiftPower(double power) {
        dumpLiftLeft.setPower(power);
        dumpLiftRight.setPower(power);
    }

    public void liftDown() {
        liftDown = true;
        mode = Mode.MOVE_DOWN;
        setLiftPower(LIFT_POWER_DOWN);
    }

    public void liftUp() {
        liftDown = false;
        mode = Mode.MOVE_UP;
        setDumpRotation(0.5);
        setLiftPower(LIFT_POWER_UP);
    }

    public boolean isLiftDown() {
        return liftDown;
    }

    private void setDumpRotation(double rot) {
        double servoPosition = LEFT_ROTATE_DOWN_POS + rot * (LEFT_ROTATE_UP_POS - LEFT_ROTATE_DOWN_POS);
        dumpRotateLeft.setPosition(servoPosition);
        dumpRotateRight.setPosition(1 - servoPosition);
    }

    private void engageRelease() {
        if (!releaseEngaged) {
            dumpRelease.setPosition(RELEASE_ENGAGE_POS);
            releaseEngaged = true;
        }
    }

    private void disengageRelease() {
        if (releaseEngaged) {
            dumpRelease.setPosition(RELEASE_DISENGAGE_POS);
            releaseEngaged = false;
        }
    }

    public boolean isDumping() {
        return liftDumping;
    }

    public void dump() {
        if (!liftDumping) {
            liftDumping = true;
            setDumpRotation(1);
            disengageRelease();
        }
    }

    public void retract() {
        if (liftDumping) {
            liftDumping = false;
            if (liftDown) {
                setDumpRotation(0);
            } else {
                setDumpRotation(0.5);
            }
            engageRelease();
        }
    }

    public void update() {
        telemetry.addData("dumpMode", mode);
        switch (mode) {
            case NORMAL:
                // do nothing
                break;
            case MOVE_DOWN:
                boolean bottomTouch = !dumpLiftBottomTouch.getState();
                telemetry.addData("dumpLiftButtonTouch", bottomTouch);
                if (bottomTouch) {
                    mode = Mode.NORMAL;
                    setLiftPower(0);
                    setDumpRotation(0);
                }
                break;
            case MOVE_UP:
                boolean topTouch = !dumpLiftTopTouch.getState();
                telemetry.addData("dumpLiftTopTouch", topTouch);
                if (topTouch) {
                    mode = Mode.NORMAL;
                    setLiftPower(0);
                }
                break;
        }
    }
}
