package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose;
import com.acmerobotics.library.localization.Twist;
import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    public static final double IN_PER_REV = 4 * Math.PI;
    public static final double TICK_PER_REV = 90 * 4 * 60;
    public static final double TICKS_PER_INCH = TICK_PER_REV / IN_PER_REV;

    private Servo leftGrabber, rightGrabber;
    private DcMotor grabberMotor, leftFront, rightFront, leftRear, rightRear;

    BNO055IMU imu;

    private Pose pose;
    double lastHeading = 0;
    double leftLast = 0;
    double rightLast = 0;

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

        pose = new Pose(0, 0, new Angle(0));

        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu = hardwareMap.get(BNO055IMU.class, "imu");
        imu.initialize(parameters);

        lastHeading = imu.getAngularOrientation().firstAngle;
        rightLast = rightRear.getCurrentPosition() / TICKS_PER_INCH;
        leftLast = leftRear.getCurrentPosition() / TICKS_PER_INCH;
    }

    @Override
    public void loop() {

        double heading = imu.getAngularOrientation().firstAngle;
        double left = leftFront.getCurrentPosition() / TICKS_PER_INCH;
        double right = rightFront.getCurrentPosition() / TICKS_PER_INCH;
        double dl = left - leftLast;
        double dr = right - rightLast;
        Twist update = Twist.fromArcHeading((dl + dr) / 2, new Angle(lastHeading), new Angle(heading));
        lastHeading = heading;
        leftLast = left;
        rightLast = right;

        pose.addTwist(update);
        telemetry.addData("encoderLeft", left);
        telemetry.addData("encoderRight", right);
        telemetry.addData("position", pose.x() + ", " + pose.y());
        telemetry.addData("speeds", (dl + dr) / 2);
        telemetry.addData("heading", pose.theta().value());

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
