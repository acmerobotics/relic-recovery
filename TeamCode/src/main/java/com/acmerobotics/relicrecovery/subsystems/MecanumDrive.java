package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.acmerobotics.relicrecovery.hardware.MaxSonarEZ1UltrasonicSensor;
import com.acmerobotics.relicrecovery.localization.DeadReckoningLocalizer;
import com.acmerobotics.relicrecovery.localization.Localizer;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.path.Path;
import com.acmerobotics.relicrecovery.path.PathFollower;
import com.acmerobotics.relicrecovery.util.DrawingUtil;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
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
    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(48.0, 96.0, 96.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(4.0, 8.0, 8.0, MotionConstraints.EndBehavior.OVERSHOOT);

    public static PIDFCoefficients HEADING_PID = new PIDFCoefficients(-1, 0, 0, 0.232, 0.04);
    public static PIDFCoefficients AXIAL_PID = new PIDFCoefficients(-0.02, 0, 0, 0.0182, 0.004);
    public static PIDFCoefficients LATERAL_PID = new PIDFCoefficients(-0.02, 0, 0, 0.0185, 0.004);

    public static PIDCoefficients COLUMN_ALIGN_PID = new PIDCoefficients(0.06, 0, 0.04);
    public static double COLUMN_ALIGN_TARGET_DISTANCE = 3.5;
    public static double COLUMN_ALIGN_ALLOWED_ERROR = 0.5;
    public static double SIDE_DISTANCE_SMOOTHING_COEFF = 0.1;

    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(0, 0, 0);

    public static double RAMP_MAX_ACCEL = 25;

    public enum Mode {
        OPEN_LOOP,
        OPEN_LOOP_RAMP,
        FOLLOW_PATH,
        COLUMN_ALIGN
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

    private MaxSonarEZ1UltrasonicSensor ultrasonic;

    private LynxI2cColorRangeSensor sideColorDistance;
    private ExponentialSmoother sideDistanceSmoother;
    private PIDController columnAlignController;

    private boolean useCachedOrientation;
    private Orientation cachedOrientation;
    private double headingOffset;

    private boolean positionEstimationEnabled;
    private Pose2d estimatedPose = new Pose2d(0, 0, 0);

    private PathFollower pathFollower;

    private Localizer localizer;

    private PIDController maintainHeadingController;
    private boolean maintainHeading;

    private Mode mode = Mode.OPEN_LOOP;

    private double[] powers, targetPowers;
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
            MOTOR_NAMES[3] + "Rotation",
            "sideDistance",
            "columnAlignError",
            "columnAlignUpdate"
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

        imu = LynxOptimizedI2cSensorFactory.createLynxEmbeddedIMU(map.get(LynxModule.class, "frontHub"), 1);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        ultrasonic = new MaxSonarEZ1UltrasonicSensor(map.analogInput.get("ultrasonic"));
        sideColorDistance = map.get(LynxI2cColorRangeSensor.class, "sideColorDistance");

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

        localizer = new DeadReckoningLocalizer(this);
        setEstimatedPose(estimatedPose);

        pathFollower = new PathFollower(HEADING_PID, AXIAL_PID, LATERAL_PID);
        maintainHeadingController = new PIDController(MAINTAIN_HEADING_PID);
        columnAlignController = new PIDController(COLUMN_ALIGN_PID);

        sideDistanceSmoother = new ExponentialSmoother(SIDE_DISTANCE_SMOOTHING_COEFF);

        resetEncoders();
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public void setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        localizer.setEstimatedPosition(estimatedPose.pos());
    }

    public PathFollower getPathFollower() {
        return pathFollower;
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
        this.mode = mode;
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
        return getAngularOrientation().firstAngle;
    }

    public double getHeading() {
        return Angle.norm(getRawHeading() + headingOffset);
    }

    public void resetHeading() {
        setHeading(0);
    }

    public void followPath(Path path) {
        if (!positionEstimationEnabled) {
            throw new IllegalStateException("SlapperPosition estimation must be enable for path following");
        }
        pathFollower.follow(path);
        setMode(Mode.FOLLOW_PATH);
    }

    public boolean isFollowingPath() {
        return pathFollower.isFollowingPath();
    }

    public Vector2d getEstimatedPosition() {
        return estimatedPose.pos();
    }

    public void setEstimatedPosition(Vector2d position) {
        estimatedPose = new Pose2d(position, estimatedPose.heading());
        localizer.setEstimatedPosition(position);
    }

    public Pose2d getEstimatedPose() {
        return estimatedPose;
    }

    public void setEstimatedPose(Pose2d pose) {
        setEstimatedPosition(pose.pos());
        setHeading(pose.heading());
    }

    private void invalidateOrientation() {
        useCachedOrientation = false;
    }

    public void alignWithColumn() {
        columnAlignController.reset();
        columnAlignController.setSetpoint(COLUMN_ALIGN_TARGET_DISTANCE);
        sideDistanceSmoother.reset();
        setMode(Mode.COLUMN_ALIGN);
    }

    public double getUltrasonicDistance(DistanceUnit unit) {
        return ultrasonic.getDistance(unit);
    }

    public double getMinUltrasonicDistance(DistanceUnit unit) {
        return ultrasonic.getMinDistance(unit);
    }

    public double getSideDistance(DistanceUnit unit) {
        double sideDistance = sideColorDistance.getDistance(unit);
        return Double.isNaN(sideDistance) ? unit.fromCm(20) : sideDistance;
    }

    public void update() {
        invalidateOrientation();

        telemetryMap.put("driveMode", mode);
        telemetryMap.put("positionEstimationEnabled", positionEstimationEnabled);
        telemetryMap.put("maintainHeading", maintainHeading);

        // position estimation
        if (positionEstimationEnabled) {
            estimatedPose = new Pose2d(localizer.update(), getHeading());
        }

        // maintain heading
        if (maintainHeading) {
            double heading = getHeading();
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
                } else {
                    stop();
                }
                powers = targetPowers;
                break;
            case COLUMN_ALIGN:
                double sideDistance = sideDistanceSmoother.update(
                        sideColorDistance.getDistance(DistanceUnit.INCH));
                double distanceError = columnAlignController.getError(sideDistance);

                telemetryMap.put("sideDistance", sideDistance);
                telemetryMap.put("columnAlignError", distanceError);

                if (Math.abs(distanceError) > COLUMN_ALIGN_ALLOWED_ERROR) {
                    double lateralUpdate = columnAlignController.update(distanceError);
                    internalSetVelocity(new Vector2d(0, lateralUpdate), 0);

                    telemetryMap.put("columnAlignUpdate", lateralUpdate);
                } else {
                    stop();
                }
                powers = targetPowers;
                break;
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
            telemetryMap.put(MOTOR_NAMES[i] + "Power", powers[i]);
        }

        telemetryMap.put("estimatedX", estimatedPose.x());
        telemetryMap.put("estimatedY", estimatedPose.y());
        telemetryMap.put("estimatedHeading", estimatedPose.heading());

        if (pathFollower.getPath() != null) {
            fieldOverlay.setStroke("#4CAF50");
            DrawingUtil.drawPath(fieldOverlay, pathFollower.getPath());
        }

        if (pathFollower.getPose() != null) {
            fieldOverlay.setStroke("#F44336");
            DrawingUtil.drawMecanumRobot(fieldOverlay, pathFollower.getPose());
        }

        fieldOverlay.setStroke("#3F51B5");
        DrawingUtil.drawMecanumRobot(fieldOverlay, estimatedPose);

        for (Map.Entry<String, Object> entry : telemetryMap.entrySet()) {
            telemetry.addData(entry.getKey(), entry.getValue());
        }
    }
}
