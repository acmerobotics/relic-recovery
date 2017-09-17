package com.acmerobotics.velocityvortex.mech;

import com.acmerobotics.velocityvortex.sensors.AverageDifferentiator;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.VoltageSensor;
import com.qualcomm.robotcore.util.Range;

/**
 * @author Ryan Brott
 */

public class FixedLauncher {

    public static final double TRIGGER_UP = .97;
    public static final double TRIGGER_DOWN = .7;

    public static final int DIFF_MS = 1000;

    private AverageDifferentiator leftSpeedMeasurer, rightSpeedMeasurer;

    private Servo trigger;
    private boolean triggered;
    private DcMotor left, right;
    private double leftPower, rightPower, trim;

    private double leftTarget, rightTarget;
    private long stopTime, lastTime;
    private boolean ramping;
    private int leftInitialPos, rightInitialPos;

    private VoltageSensor voltageSensor;

    public FixedLauncher(HardwareMap hardwareMap) {
        leftSpeedMeasurer = new AverageDifferentiator(DIFF_MS);
        rightSpeedMeasurer = new AverageDifferentiator(DIFF_MS);

        voltageSensor = hardwareMap.voltageSensor.get("launcher");

        trigger = hardwareMap.servo.get("trigger");
        trigger.setPosition(TRIGGER_DOWN);

        left = hardwareMap.dcMotor.get("launcherLeft");
        right = hardwareMap.dcMotor.get("launcherRight");

        left.setDirection(DcMotorSimple.Direction.REVERSE);
        left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        reset();
    }

    public void setMode(DcMotor.RunMode mode) {
        left.setMode(mode);
        right.setMode(mode);
    }

    public void delay(int ms) {
        long startTime = System.currentTimeMillis();
        while ((System.currentTimeMillis() - startTime) < ms) {
            Thread.yield();
        }
    }

    public void triggerUp() {
        if (!triggered) {
            trigger.setPosition(TRIGGER_UP);
            triggered = true;
        }
    }

    public void triggerDown() {
        if (triggered) {
            trigger.setPosition(TRIGGER_DOWN);
            triggered = false;
        }
    }

    public void triggerToggle() {
        if (triggered) {
            triggerDown();
        } else {
            triggerUp();
        }
    }

    public double getLeftSpeed() {
        return leftSpeedMeasurer.getLastDerivative();
    }

    public double getRightSpeed() {
        return rightSpeedMeasurer.getLastDerivative();
    }

    public boolean isBusy() {
        return ramping;
    }

    public boolean isRunning() {
        return leftTarget != 0 || rightTarget != 0;
    }

    public boolean isTriggered() {
        return triggered;
    }

    public double getTrim() {
        return trim;
    }

    public void setTrim(double trim) {
        this.trim = trim;
        setPower(leftPower, rightPower);
    }

    public void setPower(double power) {
        setPower(power, power);
    }

    public void setPower(double leftPower, double rightPower) {
        setPower(leftPower, rightPower, 0);
    }

    public void setPower(double leftPower, double rightPower, long rampTime) {
        if (leftPower == 0) {
            leftTarget = 0;
        } else {
            leftTarget = leftPower - trim;
        }
        if (rightPower == 0) {
            rightTarget = 0;
        } else {
            rightTarget = rightPower + trim;
        }

        if (rampTime == 0) {
            internalSetPower(leftTarget, rightTarget);
        } else {
            ramping = true;
            lastTime = System.currentTimeMillis();
            stopTime = lastTime + rampTime;
            if (leftTarget == 0) {
                internalSetLeftPower(leftPower);
            }
            if (rightTarget == 0) {
                internalSetRightPower(rightPower);
            }
        }
    }

    public void update() {
        long now = System.currentTimeMillis();

        double leftSpeed = leftSpeedMeasurer.update(getLeftPosition());
        double rightSpeed = rightSpeedMeasurer.update(getRightPosition());

        if (ramping) {
            if (now >= stopTime) {
                ramping = false;
                internalSetPower(leftTarget, rightTarget);
            } else {
                long smallDelta = now - lastTime;
                long bigDelta = stopTime - lastTime;
                if (leftTarget != 0) {
                    internalSetLeftPower(leftPower + smallDelta * (leftTarget - leftPower) / bigDelta);
                }
                if (rightTarget != 0) {
                    internalSetRightPower(rightPower + smallDelta * (rightTarget - rightPower) / bigDelta);
                }
                lastTime = now;
            }
        }
    }

    private void internalSetPower(double leftPower, double rightPower) {
        internalSetLeftPower(leftPower);
        internalSetRightPower(rightPower);
    }

    private void internalSetLeftPower(double power) {
        this.leftPower = Range.clip(power, -1, 1);
        left.setPower(leftPower);
    }

    private void internalSetRightPower(double power) {
        this.rightPower = Range.clip(power, -1, 1);
        right.setPower(rightPower);
    }

    public int getLeftPosition() {
        return left.getCurrentPosition() - leftInitialPos;
    }

    public int getRightPosition() {
        return right.getCurrentPosition() - rightInitialPos;
    }

    public void reset() {
        leftInitialPos = left.getCurrentPosition();
        rightInitialPos = right.getCurrentPosition();
    }

    public double getLeftPower() {
        return leftPower;
    }

    public double getRightPower() {
        return rightPower;
    }

    @Deprecated
    public void fireBalls(int numBalls, LinearOpMode opMode) {
        if (numBalls == 0) return;

        setPower(1, 1, 2000);

        long startTime = System.currentTimeMillis();
        while ((opMode == null || opMode.opModeIsActive()) && (System.currentTimeMillis() - startTime) < 2500) {
            update();
            Thread.yield();
        }

        for (int i = 1; i <= numBalls; i++) {
            triggerUp();
            delay(500);
            triggerDown();
            if (i == numBalls) {
                setPower(0);
                return;
            }
            delay(2500);
        }
    }

    public double getVoltage() {
        return voltageSensor.getVoltage();
    }

}
