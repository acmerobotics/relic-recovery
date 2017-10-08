package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * @author Ryan
 */

@TeleOp
public class Ri24hTeleOp extends OpMode {

    public static final double LEFT_OPEN = 0.500;
    public static final double RIGHT_OPEN = 0.350;
    public static final double LEFT_CLOSED = 0;
    public static final double RIGHT_CLOSED = 0.750;

    private Servo leftGrabber, rightGrabber;
    private DcMotor grabberMotor, leftFront, rightFront, leftRear, rightRear;

    private StickyGamepad stickyGamepad1, stickyGamepad2;
    private boolean grabberClosed, grabberHalfSpeed, driveHalfSpeed;

    @Override
    public void init() {
        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        leftFront = hardwareMap.dcMotor.get("leftFront");
        leftRear = hardwareMap.dcMotor.get("leftRear");
        rightFront = hardwareMap.dcMotor.get("rightFront");
        rightRear = hardwareMap.dcMotor.get("rightRear");

        leftRear.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFront.setDirection(DcMotorSimple.Direction.REVERSE);

        grabberMotor = hardwareMap.dcMotor.get("grabberMotor");

        leftGrabber = hardwareMap.servo.get("leftGrabber");
        rightGrabber = hardwareMap.servo.get("rightGrabber");
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        stickyGamepad2.update();

        if (stickyGamepad1.y) {
            driveHalfSpeed = !driveHalfSpeed;
        }

        if (stickyGamepad2.y) {
            grabberHalfSpeed = !grabberHalfSpeed;
        }

        // simple arcade drive
        double axial = -gamepad1.left_stick_y;
        double lateral = -gamepad1.right_stick_x;

        double speedMultiplier = driveHalfSpeed ? 0.5 : 1;

        if (axial == 0 && lateral == 0) {
            axial = -gamepad2.right_stick_y;
            lateral = -gamepad2.right_stick_x;
            speedMultiplier = 1.0 / 3.0;
        }

            double sum = Math.abs(axial + lateral);
            if (sum > 1) {
                axial /= sum;
                lateral /= sum;
            }

            double leftSpeed = axial - lateral;
            double rightSpeed = axial + lateral;

            leftSpeed *= speedMultiplier;
            rightSpeed *= speedMultiplier;

            leftFront.setPower(leftSpeed);
            leftRear.setPower(leftSpeed);
            rightFront.setPower(rightSpeed);
            rightRear.setPower(rightSpeed);

            if (stickyGamepad2.a) {
                grabberClosed = !grabberClosed;
            }

            if (grabberClosed) {
            leftGrabber.setPosition(LEFT_CLOSED);
            rightGrabber.setPosition(RIGHT_CLOSED);
        } else {
            leftGrabber.setPosition(LEFT_OPEN);
            rightGrabber.setPosition(RIGHT_OPEN);
        }

        grabberMotor.setPower(-gamepad2.left_stick_y * (grabberHalfSpeed ? 0.5 : 1));
    }
}
