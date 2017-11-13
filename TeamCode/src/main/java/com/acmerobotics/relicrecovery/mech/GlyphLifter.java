package com.acmerobotics.relicrecovery.mech;

import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.MotionGoal;
import com.acmerobotics.relicrecovery.motion.MotionProfile;
import com.acmerobotics.relicrecovery.motion.MotionProfileGenerator;
import com.acmerobotics.relicrecovery.motion.MotionState;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.motion.PIDFController;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Created by ryanbrott on 11/12/17.
 */

public class GlyphLifter implements Loop {
    public static MotionConstraints GLYPH_MOTION_CONSTRAINTS = new MotionConstraints(0, 0, 0, MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_A);
    public static PIDFCoefficients GLYPH_PIDF_COEFF = new PIDFCoefficients(0, 0, 0, 0, 0);

    public static double BALL_SCREW_THROW = 11;
    public static double RACK_THROW = 8;

    public static double REV_PER_IN = 2;

    public static double ZERO_BALL_SCREW_POWER = 0.5;
    public static double PINION_POWER = 1;

    public enum Side {
        FRONT,
        REAR
    }

    public enum Mode {
        OPEN_LOOP,
        FOLLOW_PROFILE,
        ZERO
    }

    private DcMotor ballScrewMotor;
    private CRServo pinionServo, leftIntake, rightIntake;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private DigitalChannel ballScrewTouch, lowerRackTouch, upperRackTouch;

    private double ballScrewPower, pinionPower;

    private Mode mode;

    private long profileStartTimestamp;
    private boolean extendRack;
    private MotionProfile profile;
    private PIDFController controller;

    private int encoderOffset;

    public GlyphLifter(HardwareMap hardwareMap, Side side) {
        if (side == Side.FRONT) {
            ballScrewMotor = hardwareMap.dcMotor.get("frontBallScrew");
            pinionServo = hardwareMap.crservo.get("frontPinion");
            leftIntake = hardwareMap.crservo.get("frontLeftIntake");
            rightIntake = hardwareMap.crservo.get("frontRightIntake");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "frontColorDistanceSensor");
            colorSensor = hardwareMap.get(ColorSensor.class, "frontColorDistanceSensor");
            ballScrewTouch = hardwareMap.digitalChannel.get("frontBallScrewTouch");
            lowerRackTouch = hardwareMap.digitalChannel.get("frontLowerRackTouch");
            upperRackTouch = hardwareMap.digitalChannel.get("frontUpperRackTouch");
        } else {
            ballScrewMotor = hardwareMap.dcMotor.get("rearBallScrew");
            pinionServo = hardwareMap.crservo.get("rearPinion");
            leftIntake = hardwareMap.crservo.get("rearLeftIntake");
            rightIntake = hardwareMap.crservo.get("rearRightIntake");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "rearColorDistanceSensor");
            colorSensor = hardwareMap.get(ColorSensor.class, "rearColorDistanceSensor");
            ballScrewTouch = hardwareMap.digitalChannel.get("rearBallScrewTouch");
            lowerRackTouch = hardwareMap.digitalChannel.get("rearLowerRackTouch");
            upperRackTouch = hardwareMap.digitalChannel.get("rearUpperRackTouch");
        }

        ballScrewMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        controller = new PIDFController(GLYPH_PIDF_COEFF);

        mode = Mode.OPEN_LOOP;
    }

    private int getRawEncoderPosition() {
        return ballScrewMotor.getCurrentPosition();
    }

    public void resetEncoder() {
        encoderOffset = -getRawEncoderPosition();
    }

    public int getEncoderPosition() {
        return encoderOffset + getRawEncoderPosition();
    }

    public void setBallScrewPower(double power) {
        ballScrewPower = power;
    }

    public void setPinionPower(double power) {
        pinionPower = power;
    }

    public void setLiftPower(double ballScrewPower, double pinionPower) {
        setBallScrewPower(ballScrewPower);
        setPinionPower(pinionPower);
    }

    public double getBallScrewHeight() {
        double ticks = getEncoderPosition();
        double revs = ticks / ballScrewMotor.getMotorType().getTicksPerRev();
        return revs / REV_PER_IN;
    }

    public void setHeight(double height) {
        if (0 < height || height > (BALL_SCREW_THROW + RACK_THROW)) {
            throw new IllegalArgumentException("height must be positive and smaller than the total throw");
        }

        if (height > BALL_SCREW_THROW) {
            height -= RACK_THROW;
            extendRack = true;
        } else {
            extendRack = false;
        }

        MotionState start = new MotionState(getBallScrewHeight(), 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(height, 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, GLYPH_MOTION_CONSTRAINTS);
        profileStartTimestamp = System.currentTimeMillis();
        mode = Mode.FOLLOW_PROFILE;
    }

    @Override
    public void onLoop(long timestamp, long dt) {
        switch (mode) {
            case OPEN_LOOP:
                break;
            case FOLLOW_PROFILE:
                // ball screw
                double time = (timestamp - profileStartTimestamp) / 1000.0;
                MotionState state = profile.get(time);
                controller.setSetpoint(state);
                double actualHeight = getBallScrewHeight();
                double heightError = controller.getPositionError(actualHeight);
                double heightUpdate = controller.update(heightError);
                setBallScrewPower(heightUpdate);
                // rack and pinion
                if (extendRack && !upperRackTouch.getState()) {
                    pinionServo.setPower(PINION_POWER);
                } else if (!extendRack && !lowerRackTouch.getState()) {
                    pinionServo.setPower(-PINION_POWER);
                } else {
                    pinionServo.setPower(0);
                }
                break;
            case ZERO:
                boolean ballScrewZeroed = ballScrewTouch.getState();
                boolean rackZeroed = lowerRackTouch.getState();
                if (ballScrewZeroed && rackZeroed) {
                    resetEncoder();
                    mode = Mode.OPEN_LOOP;
                }
                setLiftPower(ballScrewZeroed ? 0 : -ZERO_BALL_SCREW_POWER, rackZeroed ? 0 : -PINION_POWER);
                break;
        }

        ballScrewMotor.setPower(ballScrewPower);
        pinionServo.setPower(pinionPower);
    }
}
