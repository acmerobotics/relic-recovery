package com.acmerobotics.relicrecovery.subsystems;

import android.util.Log;

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
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.Range;

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
    public static final int IMU_PORT = 1;

    public static MotionConstraints AXIAL_CONSTRAINTS = new MotionConstraints(36.0, 24.0, 48.0, MotionConstraints.EndBehavior.OVERSHOOT);
    public static MotionConstraints POINT_TURN_CONSTRAINTS = new MotionConstraints(2.0, 2.67, 10.67, MotionConstraints.EndBehavior.OVERSHOOT);

    public static PIDFCoefficients HEADING_PID = new PIDFCoefficients(-0.5, 0, 0, 0.273, 0); // -0.5, 0.126);
    public static PIDFCoefficients AXIAL_PID = new PIDFCoefficients(-0.0001, 0, 0, 0.018, 0); // -0.02, 0.0112);
    public static PIDFCoefficients LATERAL_PID = new PIDFCoefficients(-0.0001, 0, 0, 0.0202, 0); // -0.02, 0.0138);

    public static PIDCoefficients COLUMN_ALIGN_PID = new PIDCoefficients(-0.03, 0, -0.02);
    public static double COLUMN_ALIGN_TARGET_DISTANCE = 7;
    public static double COLUMN_ALIGN_ALLOWED_ERROR = 0.5;

    public static double PROXIMITY_SMOOTHING_COEFF = 0.1;
    public static double PROXIMITY_SWIVEL_EXTEND = 0.08;
    public static double PROXIMITY_SWIVEL_RETRACT = 0.59;

    public static double ULTRASONIC_SWIVEL_EXTEND = 0.2;
    public static double ULTRASONIC_SWIVEL_RETRACT = 0.7;

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

    private boolean useCachedEncoderPositions;
    private int[] cachedEncoderPositions;

    private BNO055IMU imu;

    private MaxSonarEZ1UltrasonicSensor ultrasonicSensor;

    private Servo proximitySwivel;
    private boolean proximitySwivelExtended;

    private Servo ultrasonicSwivel;
    private boolean ultrasonicSwivelExtended;

    private LynxI2cColorRangeSensor proximitySensor;
    private ExponentialSmoother proximitySmoother;
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

    private LynxModule frontHub;

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

        public double proximityDistance;
        public double columnAlignError;
        public double columnAlignUpdate;

        public boolean proximitySwivelExtended;
        public boolean ultrasonicSwivelExtended;
    }

    public MecanumDrive(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new TelemetryEx(telemetry);
        this.telemetryData = new TelemetryData();

        this.fieldOverlay = RobotDashboard.getInstance().getFieldOverlay();

        frontHub = map.get(LynxModule.class, "frontHub");
        imu = LynxOptimizedI2cSensorFactory.createLynxEmbeddedIMU(frontHub, IMU_PORT);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        proximitySensor = map.get(LynxI2cColorRangeSensor.class, "proximitySensor");
        proximitySwivel = map.servo.get("proximitySwivel");

        ultrasonicSensor = new MaxSonarEZ1UltrasonicSensor(map.analogInput.get("ultrasonicSensor"));
        ultrasonicSwivel = map.servo.get("ultrasonicSwivel");

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

        localizer = new DeadReckoningLocalizer(this);
        setEstimatedPose(estimatedPose);

        pathFollower = new PathFollower(HEADING_PID, AXIAL_PID, LATERAL_PID);
        maintainHeadingController = new PIDController(MAINTAIN_HEADING_PID);
        maintainHeadingController.setInputBounds(-Math.PI, Math.PI);

        columnAlignController = new PIDController(COLUMN_ALIGN_PID);
        columnAlignController.setOutputBounds(-0.2, 0.2);

        proximitySmoother = new ExponentialSmoother(PROXIMITY_SMOOTHING_COEFF);

        resetEncoders();
        retractProximitySwivel();
        retractUltrasonicSwivel();
    }

    public Localizer getLocalizer() {
        return localizer;
    }

    public void setLocalizer(Localizer localizer) {
        this.localizer = localizer;
        localizer.setEstimatedPosition(estimatedPose.pos());
    }

    public void extendProximitySwivel() {
        proximitySwivelExtended = true;
    }

    public void retractProximitySwivel() {
        proximitySwivelExtended = false;
    }

    public void extendUltrasonicSwivel() {
        ultrasonicSwivelExtended = true;
    }

    public void retractUltrasonicSwivel() {
        ultrasonicSwivelExtended = false;
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
        int[] positions = internalGetEncoderPositions();
        for(int i = 0; i < 4; i++) {
            encoderOffsets[i] = -positions[i];
        }
    }

    /** @return motor rotations in radians */
    public double[] getMotorRotations() {
        double[] motorRotations = new double[4];
        int[] encoderPositions = getEncoderPositions();
        for (int i = 0; i < 4; i++) {
            motorRotations[i] = ticksToRadians(i, encoderPositions[i]);
        }
        return motorRotations;
    }

    /** @return motor positions in encoder ticks */
    public int[] getEncoderPositions() {
        if (!useCachedEncoderPositions || cachedEncoderPositions == null) {
            cachedEncoderPositions = internalGetEncoderPositions();
            for (int i = 0; i < 4; i++) {
                cachedEncoderPositions[i] += encoderOffsets[i];
            }
            useCachedEncoderPositions = true;
        }
        return cachedEncoderPositions;
    }

    private int[] internalGetEncoderPositions() {
        LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(frontHub);
        int[] positions = new int[4];
        try {
            LynxGetBulkInputDataResponse response = command.sendReceive();
            for (int i = 0; i < 4; i++) {
                positions[i] = response.getEncoder(i);
            }
            positions[2] = -positions[2];
            positions[3] = -positions[3];
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        } catch (Exception e) {
            Log.w("MecanumDrive", e);
        }
        StringBuilder s = new StringBuilder();
        for (int i = 0; i < 4; i++) {
            s.append(positions[i] + " ");
        }
        System.out.println(s.toString());
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
        if (!useCachedOrientation || cachedOrientation == null) {
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

    private void invalidateCaches() {
        useCachedOrientation = false;
        useCachedEncoderPositions = false;
    }

    public void alignWithColumn() {
        columnAlignController.reset();
        columnAlignController.setSetpoint(COLUMN_ALIGN_TARGET_DISTANCE);
        proximitySmoother.reset();
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
        return ultrasonicSensor.getDistance(unit);
    }

    public double getMinUltrasonicDistance(DistanceUnit unit) {
        return ultrasonicSensor.getMinDistance(unit);
    }

    public double getSideDistance(DistanceUnit unit) {
        return proximitySensor.getDistance(unit);
    }

    public void update() {
        invalidateCaches();

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
                double rawSideDistance = getSideDistance(DistanceUnit.INCH);
                rawSideDistance = Double.isNaN(rawSideDistance) ? 10 : Range.clip(rawSideDistance, -10, 10);

                double sideDistance = proximitySmoother.update(rawSideDistance);
                double distanceError = columnAlignController.getError(sideDistance);

                telemetryData.proximityDistance = sideDistance;
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

        // proximity swivel
        if (proximitySwivelExtended) {
            proximitySwivel.setPosition(PROXIMITY_SWIVEL_EXTEND);
        } else {
            proximitySwivel.setPosition(PROXIMITY_SWIVEL_RETRACT);
        }
        telemetryData.proximitySwivelExtended = proximitySwivelExtended;

        if (ultrasonicSwivelExtended) {
            ultrasonicSwivel.setPosition(ULTRASONIC_SWIVEL_EXTEND);
        } else {
            ultrasonicSwivel.setPosition(ULTRASONIC_SWIVEL_RETRACT);
        }
        telemetryData.ultrasonicSwivelExtended = ultrasonicSwivelExtended;

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
