package com.acmerobotics.relicrecovery.drive;

import android.support.annotation.Nullable;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.hardware.LynxOptimizedI2cSensorFactory;
import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;

import java.util.Arrays;
import java.util.Collections;

/**
 * Created by kelly on 9/28/2017
 *
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
public class MecanumDrive implements Loop {
    public enum Mode {
        OPEN_LOOP,
        OPEN_LOOP_RAMP,
        FOLLOW_PATH,
        AUTO_BALANCE
    }

    public static final String[] MOTOR_NAMES = {"frontLeft", "rearLeft", "rearRight", "frontRight"};

    public static final double WHEELBASE_WIDTH = 18;
    public static final double WHEELBASE_HEIGHT = 18;

    /***
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
    private double headingOffset;

    private boolean positionEstimationEnabled;

    private boolean cachingEnabled;
    private Orientation cachedOrientation;
    private int[] cachedEncoderPositions;

    private PositionEstimator positionEstimator;
    private PathFollower pathFollower;

    /* inputs are pitch and roll, respectively */
    private PIDController balanceAxialController, balanceLateralController;

    private PIDController maintainHeadingController;
    private boolean maintainHeading;

    private Mode mode = Mode.OPEN_LOOP;
    private Mode lastMode;

    private double[] powers, targetPowers;
    private Vector2d targetVel = new Vector2d(0, 0);
    private double targetOmega = 0;

    private Telemetry telemetry;
    private Canvas fieldOverlay;

    private PriorityScheduler scheduler;

    public MecanumDrive(HardwareMap map, PriorityScheduler scheduler, @Nullable Telemetry telemetry) {
        this.telemetry = telemetry;
        this.fieldOverlay = RobotDashboard.getInstance().getFieldOverlay();
        this.scheduler = scheduler;

        imu = LynxOptimizedI2cSensorFactory.createLynxEmbeddedIMU(map.get(LynxModule.class, "rearHub"));
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

        positionEstimator = new PositionEstimator(this, new Vector2d(0, 0));
        pathFollower = new PathFollower(DriveConstants.HEADING_COEFFS, DriveConstants.AXIAL_COEFFS, DriveConstants.LATERAL_COEFFS);

        balanceAxialController = new PIDController(DriveConstants.BALANCE_AXIAL_COEFFS);
        balanceAxialController.setOutputBounds(-1, 1);
        balanceAxialController.setSetpoint(0);

        balanceLateralController = new PIDController(DriveConstants.BALANCE_LATERAL_COEFFS);
        balanceLateralController.setOutputBounds(-1, 1);
        balanceLateralController.setSetpoint(0);

        maintainHeadingController = new PIDController(DriveConstants.MAINTAIN_HEADING_COEFFS);

        resetEncoders();

        setHeading(0);
    }

    public void enablePositionEstimation() {
        positionEstimationEnabled = true;
    }

    public void disablePositionEstimation() {
        positionEstimationEnabled = false;
    }

    public void enableCaching() {
        cachingEnabled = true;
        scheduler.addRepeating(() -> cachedOrientation = internalGetAngularOrientation(),
                "drive: read orientation", PriorityScheduler.LOW_PRIORITY);
        scheduler.addRepeating(() -> cachedEncoderPositions = internalGetEncoderPositions(),
                "drive: read encoders", PriorityScheduler.LOW_PRIORITY);
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

    public PositionEstimator getPositionEstimator() {
        return positionEstimator;
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
    void internalSetVelocity(Vector2d vel, double omega) {
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
        if (cachingEnabled) {
            return cachedEncoderPositions;
        } else {
            return internalGetEncoderPositions();
        }
    }

    /** @return motor position in encoder ticks */
    public int getEncoderPosition(int motor) {
        if (cachingEnabled) {
            return cachedEncoderPositions[motor];
        } else {
            return internalGetEncoderPosition(motor);
        }
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
        if (cachingEnabled) {
            return cachedOrientation;
        } else {
            return imu.getAngularOrientation();
        }
    }

    private double getRawHeading() {
        return getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
    }

    public double getHeading() {
        return Angle.norm(getRawHeading() + headingOffset);
    }

    public void resetHeading() {
        setHeading(0);
    }

    public void registerLoops(Looper looper) {
        looper.addLoop(this);
    }

    public void followPath(Path path) {
        pathFollower.follow(path);
        setMode(Mode.FOLLOW_PATH);
    }

    public boolean isFollowingPath() {
        return pathFollower.isFollowingPath();
    }

    public Pose2d getEstimatedPose() {
        return new Pose2d(positionEstimator.getPosition(), getHeading());
    }

    public void setEstimatedPose(Pose2d pose) {
        positionEstimator.setPosition(pose.pos());
        setHeading(pose.heading());
    }

    public void autoBalance() {
        setMode(Mode.AUTO_BALANCE);
        balanceAxialController.reset();
        balanceLateralController.reset();
    }

    @Override
    public void onLoop(double timestamp, double dt) {
        // position estimation
        if (positionEstimationEnabled) {
            positionEstimator.update(timestamp);
        }

        Pose2d estimatedPose = getEstimatedPose();

        // maintain heading
        double heading = getHeading();
        double headingError = maintainHeadingController.getError(heading);
        double headingUpdate = 0;
        if (maintainHeading) {
            if (Math.abs(targetOmega) > 0) {
                maintainHeadingController.setSetpoint(heading);
                internalSetVelocity(targetVel, targetOmega);
            } else {
                headingUpdate = maintainHeadingController.update(headingError);
                internalSetVelocity(targetVel, headingUpdate);
            }
        } else {
            internalSetVelocity(targetVel, targetOmega);
        }

        // auto balance
        Orientation angularOrientation = getAngularOrientation().toAxesOrder(AxesOrder.XYZ);
        double pitch = angularOrientation.secondAngle;
        double roll = angularOrientation.firstAngle;

        double balanceAxialError = balanceAxialController.getError(pitch);
        double balanceLateralError = balanceLateralController.getError(roll);

        double balanceAxialUpdate = 0, balanceLateralUpdate = 0;

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
                double maxAbsPowerDelta = DriveConstants.RAMP_MAX_ACCEL * (dt / 1000.0);
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
                    Pose2d update = pathFollower.update(estimatedPose, timestamp);
                    internalSetVelocity(update.pos(), update.heading());
                } else {
                    stop();
                    revertMode();
                }
                powers = targetPowers;
                break;
            case AUTO_BALANCE:
                balanceAxialUpdate = balanceAxialController.update(balanceAxialError);
                balanceLateralUpdate = balanceLateralController.update(balanceLateralError);

                internalSetVelocity(new Vector2d(balanceAxialUpdate, balanceLateralUpdate), 0);

                powers = targetPowers;

                break;
        }

        for (int i = 0; i < 4; i++) {
            int j = i;
            scheduler.add(() -> motors[j].setPower(powers[j]), "drive: motor " + j + " set power", PriorityScheduler.HIGH_PRIORITY);
        }

        if (telemetry != null) {
            telemetry.addData("driveMode", mode);

            telemetry.addData("estimatedX", estimatedPose.x());
            telemetry.addData("estimatedY", estimatedPose.y());
            telemetry.addData("heading", estimatedPose.heading());

            telemetry.addData("headingError", headingError);
            telemetry.addData("headingUpdate", headingUpdate);

            for (int i = 0; i < 4; i++) {
                telemetry.addData("drivePower" + i, powers[i]);
                telemetry.addData("drivePosition" + i, getEncoderPosition(i));
            }

            telemetry.addData("pathHeadingError", pathFollower.getHeadingError());
            telemetry.addData("pathHeadingUpdate", pathFollower.getHeadingUpdate());

            telemetry.addData("pathAxialError", pathFollower.getAxialError());
            telemetry.addData("pathAxialUpdate", pathFollower.getAxialUpdate());

            telemetry.addData("pathLateralError", pathFollower.getLateralError());
            telemetry.addData("pathLateralUpdate", pathFollower.getLateralUpdate());

            telemetry.addData("pitch", pitch);
            telemetry.addData("roll", roll);

            telemetry.addData("balanceAxialError", balanceAxialError);
            telemetry.addData("balanceLateralError", balanceLateralError);

            telemetry.addData("balanceAxialUpdate", balanceAxialUpdate);
            telemetry.addData("balanceLateralUpdate", balanceLateralUpdate);
        }

        double robotRadius = 9;
        fieldOverlay.setStrokeWidth(4);

        Pose2d pathPose = pathFollower.getPose();
        if (pathPose != null) {
            fieldOverlay.setStroke("red");
            fieldOverlay.strokeLine(
                    pathPose.x() + 0.5 * robotRadius * Math.cos(pathPose.heading()),
                    pathPose.y() + 0.5 * robotRadius * Math.sin(pathPose.heading()),
                    pathPose.x() + robotRadius * Math.cos(pathPose.heading()),
                    pathPose.y() + robotRadius * Math.sin(pathPose.heading()));
            fieldOverlay.strokeCircle(pathPose.x(), pathPose.y(), robotRadius);
        }

        fieldOverlay.setStroke("blue");
        fieldOverlay.strokeLine(
            estimatedPose.x() + 0.5 * robotRadius * Math.cos(estimatedPose.heading()),
            estimatedPose.y() + 0.5 * robotRadius * Math.sin(estimatedPose.heading()),
            estimatedPose.x() + robotRadius * Math.cos(estimatedPose.heading()),
            estimatedPose.y() + robotRadius * Math.sin(estimatedPose.heading()));
        fieldOverlay.strokeCircle(estimatedPose.x(), estimatedPose.y(), robotRadius);
    }
}
