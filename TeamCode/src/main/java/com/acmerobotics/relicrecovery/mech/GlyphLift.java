package com.acmerobotics.relicrecovery.mech;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
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
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Created by ryanbrott on 11/12/17.
 */

@Config
public class GlyphLift implements Loop {
    public static final double SENSOR_MAX_DISTANCE = DistanceUnit.INCH.fromCm(25);

    public static MotionConstraints GLYPH_MOTION_CONSTRAINTS = new MotionConstraints(0, 0, 0, MotionConstraints.EndBehavior.VIOLATE_MAX_ABS_A);
    public static PIDFCoefficients GLYPH_PIDF_COEFF = new PIDFCoefficients(0, 0, 0, 0, 0);

    public static double LEAD_SCREW_THROW = 11;
    public static double RACK_THROW = 8;

    public static double REV_PER_IN = 2;

    public static double ZERO_LEAD_SCREW_POWER = 0.5;
    public static double PINION_POWER = 1;

    public static double SLOW_INTAKE_POWER = 0.5;
    public static double FAST_INTAKE_POWER = 1;

    public enum Side {
        FRONT,
        REAR
    }

    public enum LiftMode {
        OPEN_LOOP,
        FOLLOW_PROFILE,
        ZERO
    }

    public enum IntakeMode {
        OPEN_LOOP,
        INTAKE_GLYPH
    }

    private DcMotor leadScrewMotor;
    private CRServo pinionServo, leftIntake, rightIntake;
    private DistanceSensor distanceSensor;
    private ColorSensor colorSensor;
    private DigitalChannel leadScrewTouch, lowerRackTouch, upperRackTouch;

    private double leadScrewPower, pinionPower;
    private double leftIntakePower, rightIntakePower;

    private LiftMode liftMode;
    private IntakeMode intakeMode;

    private long profileStartTimestamp;
    private boolean extendRack;
    private MotionProfile profile;
    private PIDFController controller;

    private int encoderOffset;

    private Telemetry telemetry;

    public GlyphLift(HardwareMap hardwareMap, Telemetry telemetry, Side side) {
        this.telemetry = telemetry;

        if (side == Side.FRONT) {
            leadScrewMotor = hardwareMap.dcMotor.get("frontLeadScrew");
            leadScrewMotor.setDirection(DcMotorSimple.Direction.REVERSE);
            pinionServo = hardwareMap.crservo.get("frontPinion");
            leftIntake = hardwareMap.crservo.get("frontLeftIntake");
            leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
            rightIntake = hardwareMap.crservo.get("frontRightIntake");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "frontColorDistanceSensor");
            colorSensor = hardwareMap.get(ColorSensor.class, "frontColorDistanceSensor");
            leadScrewTouch = hardwareMap.digitalChannel.get("frontLeadScrewTouch");
            lowerRackTouch = hardwareMap.digitalChannel.get("frontLowerRackTouch");
            upperRackTouch = hardwareMap.digitalChannel.get("frontUpperRackTouch");
        } else {
            leadScrewMotor = hardwareMap.dcMotor.get("rearLeadScrew");
            pinionServo = hardwareMap.crservo.get("rearPinion");
            leftIntake = hardwareMap.crservo.get("rearLeftIntake");
            rightIntake = hardwareMap.crservo.get("rearRightIntake");
            distanceSensor = hardwareMap.get(DistanceSensor.class, "rearColorDistanceSensor");
            colorSensor = hardwareMap.get(ColorSensor.class, "rearColorDistanceSensor");
            leadScrewTouch = hardwareMap.digitalChannel.get("rearLeadScrewTouch");
            lowerRackTouch = hardwareMap.digitalChannel.get("rearLowerRackTouch");
            upperRackTouch = hardwareMap.digitalChannel.get("rearUpperRackTouch");
        }

        leadScrewMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        leadScrewTouch.setMode(DigitalChannel.Mode.INPUT);
        lowerRackTouch.setMode(DigitalChannel.Mode.INPUT);
        upperRackTouch.setMode(DigitalChannel.Mode.INPUT);

        controller = new PIDFController(GLYPH_PIDF_COEFF);

        liftMode = LiftMode.OPEN_LOOP;
        intakeMode = IntakeMode.OPEN_LOOP;
    }

    public LiftMode getLiftMode() {
        return liftMode;
    }

    public IntakeMode getIntakeMode() {
        return intakeMode;
    }

    private int getRawEncoderPosition() {
        return leadScrewMotor.getCurrentPosition();
    }

    public void resetEncoder() {
        encoderOffset = -getRawEncoderPosition();
    }

    public int getEncoderPosition() {
        return encoderOffset + getRawEncoderPosition();
    }

    public void setLeadScrewPower(double power) {
        leadScrewPower = power;
        liftMode = LiftMode.OPEN_LOOP;
    }

    public void setPinionPower(double power) {
        pinionPower = power;
        liftMode = LiftMode.OPEN_LOOP;
    }

    public void setLiftPower(double leadScrewPower, double pinionPower) {
        setLeadScrewPower(leadScrewPower);
        setPinionPower(pinionPower);
    }

    public void setIntakePower(double leftPower, double rightPower) {
        this.leftIntakePower = leftPower;
        this.rightIntakePower = rightPower;
        this.intakeMode = IntakeMode.OPEN_LOOP;
    }

    public double getLeadScrewHeight() {
        double ticks = getEncoderPosition();
        double revs = ticks / leadScrewMotor.getMotorType().getTicksPerRev();
        return revs / REV_PER_IN;
    }

    public void setHeight(double height) {
        if (0 > height || height > (LEAD_SCREW_THROW + RACK_THROW)) {
            throw new IllegalArgumentException("height must be positive and smaller than the total throw");
        }

        if (height > LEAD_SCREW_THROW) {
            height -= RACK_THROW;
            extendRack = true;
        } else {
            extendRack = false;
        }

        MotionState start = new MotionState(getLeadScrewHeight(), 0, 0, 0, 0);
        MotionGoal goal = new MotionGoal(height, 0);
        profile = MotionProfileGenerator.generateProfile(start, goal, GLYPH_MOTION_CONSTRAINTS);
        profileStartTimestamp = System.currentTimeMillis();
        liftMode = LiftMode.FOLLOW_PROFILE;
    }

    public void intakeGlyph() {
        intakeMode = IntakeMode.INTAKE_GLYPH;
    }

    public void zeroLift() {
        liftMode = LiftMode.ZERO;
    }

    public boolean isLifterZeroing() {
        return liftMode == LiftMode.ZERO;
    }

    public double getIntakeDistanceIn() {
        double distance = distanceSensor.getDistance(DistanceUnit.INCH);
        if (Double.isNaN(distance)) {
            return SENSOR_MAX_DISTANCE;
        } else {
            return Range.clip(distance, 0, SENSOR_MAX_DISTANCE);
        }
    }

    public void registerLoops(Looper looper) {
        looper.addLoop(this);
    }

    @Override
    public void onLoop(long timestamp, long dt) {
        switch (liftMode) {
            case OPEN_LOOP:
                leadScrewMotor.setPower(leadScrewPower);
                pinionServo.setPower(pinionPower);
                break;
            case FOLLOW_PROFILE:
                // lead screw
                double time = (timestamp - profileStartTimestamp) / 1000.0;
                if (time > profile.end().t) {
                    setLiftPower(0, 0);
                } else {
                    MotionState state = profile.get(time);
                    controller.setSetpoint(state);
                    double actualHeight = getLeadScrewHeight();
                    double heightError = controller.getPositionError(actualHeight);
                    double heightUpdate = controller.update(heightError);
                    leadScrewMotor.setPower(heightUpdate);
                    // rack and pinion
                    if (extendRack && !upperRackTouch.getState()) {
                        pinionServo.setPower(PINION_POWER);
                    } else if (!extendRack && !lowerRackTouch.getState()) {
                        pinionServo.setPower(-PINION_POWER);
                    } else {
                        pinionServo.setPower(0);
                    }
                }
                break;
            case ZERO:
                boolean leadScrewZeroed = leadScrewTouch.getState();
                boolean rackZeroed = lowerRackTouch.getState();
                if (leadScrewZeroed && rackZeroed) {
                    resetEncoder();
                    liftMode = LiftMode.OPEN_LOOP;
                }
                leadScrewMotor.setPower(leadScrewZeroed ? 0 : -ZERO_LEAD_SCREW_POWER);
                pinionServo.setPower(rackZeroed ? 0 : -PINION_POWER);
                break;
        }

        switch (intakeMode) {
            case OPEN_LOOP:
                leftIntake.setPower(leftIntakePower);
                rightIntake.setPower(rightIntakePower);
                break;
            case INTAKE_GLYPH:
                if (getIntakeDistanceIn() <= 2) {
                    leftIntake.setPower(0);
                    rightIntake.setPower(0);
                    intakeMode = IntakeMode.OPEN_LOOP;
                } else {
                    leftIntake.setPower(SLOW_INTAKE_POWER);
                    rightIntake.setPower(FAST_INTAKE_POWER);
                }
                break;
        }

        telemetry.addData("liftMode", liftMode);

        telemetry.addData("leadScrewTouch", leadScrewTouch.getState());
        telemetry.addData("lowerRackTouch", lowerRackTouch.getState());
        telemetry.addData("upperRackTouch", lowerRackTouch.getState());

        telemetry.addData("leadScrewPower", leadScrewPower);
        telemetry.addData("leftIntakePower", leftIntakePower);
        telemetry.addData("rightIntakePower", rightIntakePower);

        telemetry.addData("leadScrewPosition", getEncoderPosition());
        telemetry.addData("intakeDistance", getIntakeDistanceIn());

        telemetry.addData("colorSensorRed", colorSensor.red());
        telemetry.addData("colorSensorGreen", colorSensor.green());
        telemetry.addData("colorSensorBlue", colorSensor.blue());
        telemetry.addData("colorSensorAlpha", colorSensor.alpha());
    }
}
