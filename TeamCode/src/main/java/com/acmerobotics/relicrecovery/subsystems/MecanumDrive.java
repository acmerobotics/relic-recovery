package com.acmerobotics.relicrecovery.subsystems;

import android.util.Log;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.path.PathFollower;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.util.DrawingUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Collections;
import java.util.LinkedHashMap;
import java.util.Map;

/**
 * Wheel layout (top view):
 *
 *        FRONT
 * (1)\\---------//(4)
 *      |       |
 *      |       |
 *      |       |
 *      |       |
 * (2)//---------\\(3)
 *
 * the paper: http://www.chiefdelphi.com/media/papers/download/2722 (see doc/Mecanum_Kinematic_Analysis_100531.pdf)
 */
@Config
public class MecanumDrive extends Subsystem {
    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(24.0, 48.0, 48.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 4.0, 4.0, MotionConstraints.EndBehavior.OVERSHOOT);

    public static PIDFCoefficients HEADING_PID = new PIDFCoefficients(-0.01, 0, 0, 0.234, 0);
    public static PIDFCoefficients AXIAL_PID = new PIDFCoefficients(0, 0, 0, 0.0185, 0);
    public static PIDFCoefficients LATERAL_PID = new PIDFCoefficients(0, 0, 0, 0.0183, 0);

    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double RAMP_MAX_ACCEL = 25;

    public enum Mode {
        OPEN_LOOP,
        OPEN_LOOP_RAMP,
        FOLLOW_PATH
    }

    public static final String[] MOTOR_NAMES = {"frontLeft", "rearLeft", "rearRight", "frontRight"};

    public static final double WHEELBASE_WIDTH = 18;
    public static final double WHEELBASE_HEIGHT = 18;

    /**
     * K = (wheelbase width + wheelbase height) / 4 (in)
     */
    public static final double K = (WHEELBASE_WIDTH + WHEELBASE_HEIGHT) / 4;

    /**
     * wheel radius (in)
     */
    public static final double RADIUS = 2;

    private DcMotor[] motors;

    /**
     * units in encoder ticks; solely intended for internal use
     */
    private int[] encoderOffsets;

    private double lastTimestamp;

    private BNO055IMU imu;

    private boolean useCachedOrientation;
    private Orientation cachedOrientation;
    private double headingOffset;

    private boolean positionEstimationEnabled;
    private double[] lastRotations;
    private Vector2d estimatedPosition;

    private PathFollower pathFollower;

    private PIDController maintainHeadingController;
    private boolean maintainHeading;

    private Mode mode = Mode.OPEN_LOOP;
    private Mode lastMode;

    private double[] powers, targetPowers, lastPowers;
    private Vector2d targetVel = new Vector2d(0, 0);
    private double targetOmega = 0;

    public static final String[] CONDITIONAL_TELEMETRY_KEYS = {
            "pathAxialError",
            "pathAxialUpdate",
            "pathLateralError",
            "pathLateralUpdate",
            "pathHeadingError",
            "pathHeadingUpdate",
            "maintainHeadingError",
            "maintainHeadingUpdate",
            "driveHeading",
            MOTOR_NAMES[0] + "Rotation",
            MOTOR_NAMES[1] + "Rotation",
            MOTOR_NAMES[2] + "Rotation",
            MOTOR_NAMES[3] + "Rotation"
    };

    private Telemetry telemetry;
    private LinkedHashMap<String, Object> telemetryMap;

    private Canvas fieldOverlay;

    public MecanumDrive(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetryMap = new LinkedHashMap<>();
        for (String key : CONDITIONAL_TELEMETRY_KEYS) {
            telemetryMap.put(key, 0);
        }

        this.fieldOverlay = RobotDashboard.getInstance().getFieldOverlay();

        imu = LynxOptimizedI2cSensorFactory.createLynxBNO055IMU(map.get(LynxModule.class, "rearHub"), 1);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        powers = new double[4];
        targetPowers = new double[4];
        encoderOffsets = new int[4];
        motors = new DcMotor[4];
        for (int i = 0; i < 4; i ++) {
            motors[i] = map.dcMotor.get(MOTOR_NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        pathFollower = new PathFollower(HEADING_PID, AXIAL_PID, LATERAL_PID);

        estimatedPosition = new Vector2d(0, 0);

        lastPowers = new double[4];

        maintainHeadingController = new PIDController(MAINTAIN_HEADING_PID);

        resetEncoders();

        setHeading(0);
    }

    public void enablePositionEstimation() {
        positionEstimationEnabled = true;
    }

    public void disablePositionEstimation() {
        positionEstimationEnabled = false;
    }

    public DcMotor[] getMotors() {
        return motors;
    }

    public void enableHeadingCorrection() {
        maintainHeading = true;
        this.maintainHeadingController.reset();
    }

    public void disableHeadingCorrection() {
        maintainHeading = false;
    }

    public void setTargetHeading(double heading) {
        this.maintainHeadingController.setSetpoint(heading);
    }

    public boolean getMaintainHeading() {
        return maintainHeading;
    }

    private void setMode(Mode mode) {
        this.lastMode = this.mode;
        this.mode = mode;
    }

    private void revertMode() {
        this.mode = this.lastMode;
    }

    public Mode getMode() {
        return mode;
    }

    public void setHeading(double heading) {
        headingOffset = -getRawHeading() + heading;
    }

    public void setVelocity(Vector2d vel, double omega, boolean ramp) {
        this.targetVel = vel;
        this.targetOmega = omega;
        setMode(ramp ? Mode.OPEN_LOOP_RAMP : Mode.OPEN_LOOP);
    }

    public void setVelocity(Vector2d vel, double omega) {
        setVelocity(vel, omega, false);
    }

    /**
     * not as pretty but the exactly same result as the other version
     * [W] is wheel rotations
     * [V] = [vX, vY, vOmega]'
     * [R] = [1, -1, -K]
     *       [1,  1, -K]
     *       [1, -1,  K]
     *       [1,  1,  K]
     *
     * [W] = [V][R]
     *
     * @param vel
     * @param omega
     */
    // removed K's to circumvent scaling issues
    private void internalSetVelocity(Vector2d vel, double omega) {
        targetPowers[0] = vel.x() - vel.y() - omega;
        targetPowers[1] = vel.x() + vel.y() - omega;
        targetPowers[2] = vel.x() - vel.y() + omega;
        targetPowers[3] = vel.x() + vel.y() + omega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(targetPowers[0]),
                Math.abs(targetPowers[1]), Math.abs(targetPowers[2]), Math.abs(targetPowers[3])));

        for (int i = 0; i < 4; i++) {
            targetPowers[i] /= max;
        }
    }

    public void stop() {
        setVelocity(new Vector2d(0, 0), 0);
    }

    /**
     * get distance traveled from encoder values
     *
     * [W] is wheel rotations
     * [V] = [vX, vY, vOmega]'
     * [F] = [ 1/4,    1/4,   1/4,   1/4]
     *       [-1/4,    1/4,  -1/4,   1/4]
     *       [-1/4K, -1/4K,  1/4K,  1/4K]
     *
     * [V] = (r)[W][F]
     *
     * @param rot rotation of each wheel, in radians
     * @return movement of robot
     */
    public static Pose2d getPoseDelta(double[] rot) {
        if (rot.length != 4) {
            throw new IllegalArgumentException("length must be four");
        }
        double x = RADIUS * ( rot[0] + rot[1] + rot[2] + rot[3]) / 4;
        double y = RADIUS * (-rot[0] + rot[1] - rot[2] + rot[3]) / 4;
        double h = RADIUS * (-rot[0] - rot[1] + rot[2] + rot[3]) / (4 * MecanumDrive.K);
        return new Pose2d(x, y, h);
    }

    private void resetEncoders() {
        for(int i = 0; i < 4; i++) {
            encoderOffsets[i] = -motors[i].getCurrentPosition();
        }
    }

    /** @return motor rotations in radians */
    public double[] getMotorRotations() {
        double[] rotations = new double[4];
        for (int i = 0; i < 4; i++) {
            rotations[i] = getMotorRotation(i);
        }
        return rotations;
    }

    /** @return motor rotation in radians */
    public double getMotorRotation(int motor) {
        return ticksToRadians(motor, getEncoderPosition(motor));
    }

    /** @return motor positions in encoder ticks */
    public int[] getEncoderPositions() {
        return internalGetEncoderPositions();
    }

    /** @return motor position in encoder ticks */
    public int getEncoderPosition(int motor) {
        return internalGetEncoderPosition(motor);
    }

    private int internalGetEncoderPosition(int motor) {
        return encoderOffsets[motor] + motors[motor].getCurrentPosition();
    }

    private int[] internalGetEncoderPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = internalGetEncoderPosition(i);
        }
        return positions;
    }

    private double ticksToRadians(int motor, int ticks) {
        double ticksPerRev = motors[motor].getMotorType().getTicksPerRev();
        return 2 * Math.PI * ticks / ticksPerRev;
    }

    private Orientation internalGetAngularOrientation() {
        return imu.getAngularOrientation();
    }

    public Orientation getAngularOrientation() {
        if (!useCachedOrientation) {
            cachedOrientation = internalGetAngularOrientation();
            useCachedOrientation = true;
        }
        return cachedOrientation;
    }

    private double getRawHeading() {
        return -getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
    }

    public double getHeading() {
        return Angle.norm(getRawHeading() + headingOffset);
    }

    public void resetHeading() {
        setHeading(0);
    }

    public void followPath(Path path) {
        if (!positionEstimationEnabled) {
            Log.i("MecanumDrive", "Following path without position estimation enabled");
        }
        pathFollower.follow(path);
        setMode(Mode.FOLLOW_PATH);
    }

    public boolean isFollowingPath() {
        return pathFollower.isFollowingPath();
    }

    public Pose2d getEstimatedPose() {
        return new Pose2d(estimatedPosition, getHeading());
    }

    public void setEstimatedPose(Pose2d pose) {
        estimatedPosition = pose.pos();
        setHeading(pose.heading());
    }

    private void invalidateHeading() {
        useCachedOrientation = false;
    }

    public void update() {
        invalidateHeading();

        telemetryMap.put("driveMode", mode);

        if (positionEstimationEnabled || mode == Mode.FOLLOW_PATH || maintainHeading) {
            double heading = getHeading();

            // position estimation
            if (positionEstimationEnabled || mode == Mode.FOLLOW_PATH) {
                double[] rotations = getMotorRotations();
                if (lastRotations != null) {
                    double[] rotationDeltas = new double[4];
                    for (int i = 0; i < 4; i++) {
                        rotationDeltas[i] = rotations[i] - lastRotations[i];
                    }

                    Vector2d robotPoseDelta = MecanumDrive.getPoseDelta(rotationDeltas).pos();
                    Vector2d fieldPoseDelta = robotPoseDelta.rotated(heading);

                    estimatedPosition = estimatedPosition.added(fieldPoseDelta);
                }
                lastRotations = rotations;

                for (int i = 0; i < 4; i++) {
                    telemetryMap.put(MOTOR_NAMES[i] + "Rotation", rotations[i]);
                }
                telemetryMap.put("driveHeading", getHeading());
            }

            // maintain heading
            if (maintainHeading) {
                double headingError = maintainHeadingController.getError(heading);
                double headingUpdate = 0;
                if (Math.abs(targetOmega) > 0) {
                    maintainHeadingController.setSetpoint(heading);
                    internalSetVelocity(targetVel, targetOmega);
                } else {
                    headingUpdate = maintainHeadingController.update(headingError);
                    internalSetVelocity(targetVel, headingUpdate);
                }
                telemetryMap.put("maintainHeadingError", headingError);
                telemetryMap.put("maintainHeadingUpdate", headingUpdate);
            } else {
                internalSetVelocity(targetVel, targetOmega);
            }
        } else {
            internalSetVelocity(targetVel, targetOmega);
        }

        switch (mode) {
            case OPEN_LOOP:
                powers = targetPowers;
                break;
            case OPEN_LOOP_RAMP:
                double[] powerDeltas = new double[4];
                double maxDesiredAbsPowerDelta = 0;
                for (int i = 0; i < 4; i++) {
                    powerDeltas[i] = (targetPowers[i] - powers[i]);
                    double desiredAbsPowerDelta = Math.abs(powerDeltas[i]);
                    if (desiredAbsPowerDelta > maxDesiredAbsPowerDelta) {
                        maxDesiredAbsPowerDelta = desiredAbsPowerDelta;
                    }
                }

                double timestamp = TimestampedData.getCurrentTime();
                double dt = timestamp - lastTimestamp;
                lastTimestamp = timestamp;

                double maxAbsPowerDelta = RAMP_MAX_ACCEL * (dt / 1000.0);
                double multiplier;
                if (maxDesiredAbsPowerDelta > maxAbsPowerDelta) {
                    multiplier = maxAbsPowerDelta / maxDesiredAbsPowerDelta;
                } else {
                    multiplier = 1;
                }
                for (int i = 0; i < 4; i++) {
                    powers[i] += powerDeltas[i] * multiplier;
                }
                break;
            case FOLLOW_PATH:
                if (pathFollower.isFollowingPath()) {
                    Pose2d estimatedPose = getEstimatedPose();
                    Pose2d update = pathFollower.update(estimatedPose);
                    internalSetVelocity(update.pos(), update.heading());

                    telemetryMap.put("pathAxialError", pathFollower.getAxialError());
                    telemetryMap.put("pathAxialUpdate", pathFollower.getAxialUpdate());
                    telemetryMap.put("pathLateralError", pathFollower.getLateralError());
                    telemetryMap.put("pathLateralUpdate", pathFollower.getLateralUpdate());
                    telemetryMap.put("pathHeadingError", pathFollower.getHeadingError());
                    telemetryMap.put("pathHeadingUpdate", pathFollower.getHeadingUpdate());

                    fieldOverlay.setStroke("#4CAF50");
                    DrawingUtil.drawPath(fieldOverlay, pathFollower.getPath());

                    fieldOverlay.setStroke("#F44336");
                    DrawingUtil.drawMecanumRobot(fieldOverlay, pathFollower.getPose());
                } else {
                    stop();
                    revertMode();
                }
                powers = targetPowers;
                break;
        }

        for (int i = 0; i < 4; i++) {
            if (lastPowers[i] != powers[i]) {
                motors[i].setPower(powers[i]);
                lastPowers[i] = powers[i];
            }
            telemetryMap.put(MOTOR_NAMES[i] + "Power", powers[i]);
        }

//        fieldOverlay.setStroke("#3F51B5");
//        DrawingUtil.drawMecanumRobot(fieldOverlay, getEstimatedPose());

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }
}
