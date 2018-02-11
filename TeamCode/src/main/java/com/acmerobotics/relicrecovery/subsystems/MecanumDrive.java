package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.TelemetryEx;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.acmerobotics.relicrecovery.hardware.MaxSonarEZ1UltrasonicSensor;
import com.acmerobotics.relicrecovery.localization.DeadReckoningLocalizer;
import com.acmerobotics.relicrecovery.localization.Localizer;
import com.acmerobotics.relicrecovery.motion.MotionConstraints;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.motion.PIDFCoefficients;
import com.acmerobotics.relicrecovery.opmodes.AutoOpMode;
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
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Collections;

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
    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(30.0, 60.0, 120.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 4.0, 4.0, MotionConstraints.EndBehavior.OVERSHOOT);

    public static PIDFCoefficients HEADING_PID = new PIDFCoefficients(-1, 0, 0, 0.232, 0.04);
    public static PIDFCoefficients AXIAL_PID = new PIDFCoefficients(-0.02, 0, 0, 0.0182, 0.004);
    public static PIDFCoefficients LATERAL_PID = new PIDFCoefficients(-0.02, 0, 0, 0.0185, 0.004);

    public static PIDCoefficients COLUMN_ALIGN_PID = new PIDCoefficients(-0.06, 0, -0.04);
    public static double COLUMN_ALIGN_TARGET_DISTANCE = 5;
    public static double COLUMN_ALIGN_ALLOWED_ERROR = 0.5;
    public static double SIDE_DISTANCE_SMOOTHING_COEFF = 0.1;

    public static double SIDE_SWIVEL_EXTEND = 0.4;
    public static double SIDE_SWIVEL_RETRACT = 0.9;

    public static PIDCoefficients MAINTAIN_HEADING_PID = new PIDCoefficients(-2, 0, -0.01);

    public enum Mode {
        OPEN_LOOP,
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

    private BNO055IMU imu;

    private MaxSonarEZ1UltrasonicSensor ultrasonic;

    private Servo sideSwivel;
    private boolean sideSwivelExtended;

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

    private double[] powers;
    private Vector2d targetVel = new Vector2d(0, 0);
    private double targetOmega = 0;

    private Canvas fieldOverlay;

    private TelemetryEx telemetry;
    private TelemetryData telemetryData;

    public class TelemetryData {
        public Mode driveMode;

        public double frontLeftPower;
        public double rearLeftPower;
        public double rearRightPower;
        public double frontRightPower;

        public boolean positionEstimationEnabled;
        public double estimatedX;
        public double estimatedY;
        public double estimatedHeading;

        public boolean maintainHeading;
        public double maintainHeadingError;
        public double maintainHeadingUpdate;

        public double pathAxialError;
        public double pathAxialUpdate;
        public double pathLateralError;
        public double pathLateralUpdate;
        public double pathHeadingError;
        public double pathHeadingUpdate;

        public double sideDistance;
        public double columnAlignError;
        public double columnAlignUpdate;

        public boolean sideSwivelExtended;
    }

    public MecanumDrive(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new TelemetryEx(telemetry);
        this.telemetryData = new TelemetryData();

        this.fieldOverlay = RobotDashboard.getInstance().getFieldOverlay();

        imu = LynxOptimizedI2cSensorFactory.createLynxEmbeddedIMU(map.get(LynxModule.class, "frontHub"), 1);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        ultrasonic = new MaxSonarEZ1UltrasonicSensor(map.analogInput.get("ultrasonic"));
        sideColorDistance = map.get(LynxI2cColorRangeSensor.class, "sideColorDistance");

        powers = new double[4];
        encoderOffsets = new int[4];
        motors = new DcMotor[4];
        for (int i = 0; i < 4; i ++) {
            motors[i] = map.dcMotor.get(MOTOR_NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        sideSwivel = map.servo.get("sideSwivel");

        localizer = new DeadReckoningLocalizer(this);
        setEstimatedPose(estimatedPose);

        pathFollower = new PathFollower(HEADING_PID, AXIAL_PID, LATERAL_PID);
        maintainHeadingController = new PIDController(MAINTAIN_HEADING_PID);
        maintainHeadingController.setInputBounds(-Math.PI, Math.PI);

        columnAlignController = new PIDController(COLUMN_ALIGN_PID);

        sideDistanceSmoother = new ExponentialSmoother(SIDE_DISTANCE_SMOOTHING_COEFF);

        resetEncoders();
        retractSideSwivel();
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public void setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        localizer.setEstimatedPosition(estimatedPose.pos());
    }

    public void extendSideSwivel() {
        sideSwivelExtended = true;
    }

    public void retractSideSwivel() {
        sideSwivelExtended = false;
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
        enableHeadingCorrection(getHeading());
    }

    public void enableHeadingCorrection(double desiredHeading) {
        maintainHeading = true;
        this.maintainHeadingController.setSetpoint(desiredHeading);
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

    public void setVelocity(Vector2d vel, double omega) {
        internalSetVelocity(vel, omega);
        mode = Mode.OPEN_LOOP;
    }

    private void internalSetVelocity(Vector2d vel, double omega) {
        this.targetVel = vel;
        this.targetOmega = omega;
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
     */
    // removed K's to circumvent scaling issues
    private void updatePowers() {
        powers[0] = targetVel.x() - targetVel.y() - targetOmega;
        powers[1] = targetVel.x() + targetVel.y() - targetOmega;
        powers[2] = targetVel.x() - targetVel.y() + targetOmega;
        powers[3] = targetVel.x() + targetVel.y() + targetOmega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(powers[0]),
                Math.abs(powers[1]), Math.abs(powers[2]), Math.abs(powers[3])));

        for (int i = 0; i < 4; i++) {
            powers[i] /= max;
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

    public void waitForPathFollower() {
        while (!Thread.currentThread().isInterrupted() && mode == Mode.FOLLOW_PATH) {
            try {
                Thread.sleep(AutoOpMode.POLL_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
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

    public void waitForColumnAlign() {
        while (!Thread.currentThread().isInterrupted() && mode == Mode.COLUMN_ALIGN) {
            try {
                Thread.sleep(AutoOpMode.POLL_INTERVAL);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            }
        }
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

        telemetryData.driveMode = mode;
        telemetryData.positionEstimationEnabled = positionEstimationEnabled;
        telemetryData.maintainHeading = maintainHeading;

        // position estimation
        if (positionEstimationEnabled) {
            estimatedPose = new Pose2d(localizer.update(), getHeading());
        }

        switch (mode) {
            case OPEN_LOOP:
                break;
            case FOLLOW_PATH:
                if (pathFollower.isFollowingPath()) {
                    Pose2d estimatedPose = getEstimatedPose();
                    Pose2d update = pathFollower.update(estimatedPose);
                    internalSetVelocity(update.pos(), update.heading());

                    telemetryData.pathAxialError = pathFollower.getAxialError();
                    telemetryData.pathAxialUpdate = pathFollower.getAxialUpdate();
                    telemetryData.pathLateralError = pathFollower.getLateralError();
                    telemetryData.pathLateralUpdate = pathFollower.getLateralUpdate();
                    telemetryData.pathHeadingError = pathFollower.getHeadingError();
                    telemetryData.pathHeadingUpdate = pathFollower.getHeadingUpdate();
                } else {
                    stop();
                }
                break;
            case COLUMN_ALIGN:
                double sideDistance = sideDistanceSmoother.update(getSideDistance(DistanceUnit.INCH));
                double distanceError = columnAlignController.getError(sideDistance);

                telemetryData.sideDistance = sideDistance;
                telemetryData.columnAlignError = distanceError;

                if (Math.abs(distanceError) > COLUMN_ALIGN_ALLOWED_ERROR) {
                    double lateralUpdate = columnAlignController.update(distanceError);
                    internalSetVelocity(new Vector2d(0, lateralUpdate), 0);

                    telemetryData.columnAlignUpdate = lateralUpdate;
                } else {
                    stop();
                }
                break;
        }

        // maintain heading
        if (maintainHeading) {
            double heading = getHeading();
            double headingError = maintainHeadingController.getError(heading);
            double headingUpdate = maintainHeadingController.update(headingError);
            internalSetVelocity(targetVel, headingUpdate);

            telemetryData.maintainHeadingError = headingError;
            telemetryData.maintainHeadingUpdate = headingUpdate;
        } else {
            internalSetVelocity(targetVel, targetOmega);
        }

        updatePowers();

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }

        telemetryData.frontLeftPower = powers[0];
        telemetryData.rearLeftPower = powers[1];
        telemetryData.rearRightPower = powers[2];
        telemetryData.frontRightPower = powers[3];

        // side swivel
        if (sideSwivelExtended) {
            sideSwivel.setPosition(SIDE_SWIVEL_EXTEND);
        } else {
            sideSwivel.setPosition(SIDE_SWIVEL_RETRACT);
        }
        telemetryData.sideSwivelExtended = sideSwivelExtended;

        telemetryData.estimatedX = estimatedPose.x();
        telemetryData.estimatedY = estimatedPose.y();
        telemetryData.estimatedHeading = estimatedPose.heading();

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

        telemetry.addDataObject(telemetryData);
    }
}
