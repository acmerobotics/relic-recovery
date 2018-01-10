package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/9/18.
 */

@Config
public class Intake implements Loop {
    public static PIDCoefficients ROTATE_COEFF = new PIDCoefficients();
    public static double CALIBRATION_SPEED = 0.2;
    public static double ACCEPTABLE_ERROR = 40; // units: encoder ticks

    public enum Mode {
        NORMAL,
        PID,
        CALIBRATE
    }

    private PriorityScheduler scheduler;

    private Mode mode, postCalibrationMode;

    private DcMotor intakeRotate;
    private Servo intakeGripperLeft, intakeGripperRight, intakeFlipper;
    private DigitalChannel intakeTouch;

    private PIDController intakeRotateController;

    private boolean gripping, flipperEngaged, down, calibrated;

    private int encoderOffset;

    public Intake(HardwareMap map, PriorityScheduler scheduler) {
        this.scheduler = scheduler;

        intakeRotateController = new PIDController(ROTATE_COEFF);

        intakeRotate = map.dcMotor.get("intakeRotate");
        intakeGripperLeft = map.servo.get("intakeGripperLeft");
        intakeGripperRight = map.servo.get("intakeGripperRight");
        intakeFlipper = map.servo.get("intakeFlipper");
        intakeTouch = map.digitalChannel.get("intakeTouch");

        mode = Mode.NORMAL;
    }

    public void grip() {
        if (!gripping) {
            scheduler.add(() -> intakeGripperLeft.setPosition(0.24), "intake: left gripper set pos", PriorityScheduler.HIGH_PRIORITY);
            scheduler.add(() -> intakeGripperRight.setPosition(0.24), "intake: right gripper set pos", PriorityScheduler.HIGH_PRIORITY);
            gripping = true;
        }
    }

    public void release() {
        if (gripping) {
            scheduler.add(() -> intakeGripperLeft.setPosition(0.5), "intake: left gripper set pos", PriorityScheduler.HIGH_PRIORITY);
            scheduler.add(() -> intakeGripperRight.setPosition(0), "intake: right gripper set pos", PriorityScheduler.HIGH_PRIORITY);
            gripping = false;
        }
    }

    public void engageFlipper() {
        if (!flipperEngaged) {
            scheduler.add(() -> intakeFlipper.setPosition(1), "intake: flipper set pos", PriorityScheduler.HIGH_PRIORITY);
            flipperEngaged = true;
        }
    }

    public void disengageFlipper() {
        if (flipperEngaged) {
            scheduler.add(() -> intakeFlipper.setPosition(0), "intake: flipper set pos", PriorityScheduler.HIGH_PRIORITY);
            flipperEngaged = false;
        }
    }

    public void rotateUp() {
        if (down || mode == Mode.PID) {
            if (!calibrated) {
                calibrate(Mode.PID);
            } else {
                intakeRotateController.reset();
            }
            intakeRotateController.setSetpoint(0.5 * intakeRotate.getMotorType().getTicksPerRev());
            down = false;
            mode = Mode.PID;
        }
    }

    public void rotateDown() {
        if (!down || mode == Mode.PID) {
            if (!calibrated) {
                calibrate(Mode.PID);
            } else {
                intakeRotateController.reset();
            }
            intakeRotateController.setSetpoint(0);
            down = true;
            mode = Mode.PID;
        }
    }

    private void calibrate(Mode postCalibrationMode) {
        if (!calibrated && postCalibrationMode != Mode.CALIBRATE) {
            scheduler.add(() -> intakeRotate.setPower(CALIBRATION_SPEED), "intake: motor set power", PriorityScheduler.HIGH_PRIORITY);
            mode = Mode.CALIBRATE;
            this.postCalibrationMode = postCalibrationMode;
        }
    }

    @Override
    public void onLoop(double timestamp, double dt) {
        if (mode == Mode.PID) {
            scheduler.add(() -> {
                double position = intakeRotate.getCurrentPosition() - encoderOffset;
                double error = intakeRotateController.getError(position);
                if (Math.abs(error) < ACCEPTABLE_ERROR) {
                    intakeRotate.setPower(0);
                    mode = Mode.NORMAL;
                } else {
                    double update = intakeRotateController.update(error);
                    intakeRotate.setPower(update);
                }
            }, "intake: rotate PID update", PriorityScheduler.HIGH_PRIORITY);
        } else if (mode == Mode.CALIBRATE) {
            scheduler.add(() -> {
                boolean touchIsPressed = !intakeTouch.getState();
                if (touchIsPressed) {
                    intakeRotate.setPower(0);
                    calibrated = true;
                    encoderOffset = intakeRotate.getCurrentPosition();
                    mode = postCalibrationMode;
                    if (mode == Mode.PID) {
                        intakeRotateController.reset();
                    }
                }
            }, "intake: calibration update", PriorityScheduler.HIGH_PRIORITY);
        }
    }

    public void registerLoops(Looper looper) {
        looper.addLoop(this);
    }
}
