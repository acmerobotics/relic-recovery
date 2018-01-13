package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * Created by ryanbrott on 1/9/18.
 */

@Config
public class Intake {
    public static PIDCoefficients ROTATE_PID = new PIDCoefficients(-0.005, 0, 0);
    public static double CALIBRATION_SPEED = -0.2;
    public static double ACCEPTABLE_ERROR = 3; // units: encoder ticks

    public enum Mode {
        NORMAL,
        PID,
        CALIBRATE
    }

    private Mode mode, postCalibrationMode;

    private DcMotor intakeRotate;
    private Servo intakeGripperLeft, intakeGripperRight, intakeFlipper;
    private DigitalChannel intakeTouch;

    private PIDController intakeRotateController;

    private boolean closed, flipperEngaged, down, calibrated;

    private int encoderOffset;

    private Telemetry telemetry;

    public Intake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;

        intakeRotateController = new PIDController(ROTATE_PID);

        intakeRotate = map.dcMotor.get("intakeRotate");
        intakeGripperLeft = map.servo.get("intakeGripperLeft");
        intakeGripperRight = map.servo.get("intakeGripperRight");
        intakeFlipper = map.servo.get("intakeFlipper");

        intakeTouch = map.digitalChannel.get("intakeTouch");
        intakeTouch.setMode(DigitalChannel.Mode.INPUT);

        mode = Mode.NORMAL;
    }

    public boolean isClosed() {
        return closed;
    }

    public void close() {
        if (!closed) {
            intakeGripperLeft.setPosition(0.05);
            intakeGripperRight.setPosition(0.3);
            closed = true;
        }
    }

    public void open() {
        if (closed) {
            intakeGripperLeft.setPosition(0.35);
            intakeGripperRight.setPosition(0.1);
            closed = false;
        }
    }

    public void engageFlipper() {
        if (!flipperEngaged) {
            intakeFlipper.setPosition(1);
            flipperEngaged = true;
        }
    }

    public void disengageFlipper() {
        if (flipperEngaged) {
            intakeFlipper.setPosition(0);
            flipperEngaged = false;
        }
    }

    public boolean isRotatedDown() {
        return down;
    }

    public void rotateUp() {
        if (!calibrated) {
            calibrate(Mode.PID);
        } else {
            intakeRotateController.reset();
        }
        intakeRotateController.setSetpoint(0.57 * intakeRotate.getMotorType().getTicksPerRev());
        down = false;
        mode = Mode.PID;
    }

    public void rotateDown() {
        if (!calibrated) {
            calibrate(Mode.PID);
        } else {
            intakeRotateController.reset();
        }
        intakeRotateController.setSetpoint(0);
        down = true;
        mode = Mode.PID;
    }

    private void calibrate(Mode postCalibrationMode) {
        if (!calibrated && postCalibrationMode != Mode.CALIBRATE) {
            intakeRotate.setPower(CALIBRATION_SPEED);
            mode = Mode.CALIBRATE;
            this.postCalibrationMode = postCalibrationMode;
        }
    }

    public void update() {
        telemetry.addData("intakeMode", mode);
        if (mode == Mode.PID) {
            double position = intakeRotate.getCurrentPosition() - encoderOffset;
            telemetry.addData("intakePosition", position);
            double error = intakeRotateController.getError(position);
            telemetry.addData("intakeError", error);
            if (Math.abs(error) < ACCEPTABLE_ERROR) {
                intakeRotate.setPower(0);
                mode = Mode.NORMAL;
            } else {
                double update = intakeRotateController.update(error);
                intakeRotate.setPower(update);
            }
        } else if (mode == Mode.CALIBRATE) {
            boolean touchIsPressed = !intakeTouch.getState();
            telemetry.addData("intakeTouch", touchIsPressed);
            if (touchIsPressed) {
                intakeRotate.setPower(0);
                calibrated = true;
                encoderOffset = intakeRotate.getCurrentPosition();
                mode = postCalibrationMode;
                if (mode == Mode.PID) {
                    intakeRotateController.reset();
                }
            }
        }
    }
}
