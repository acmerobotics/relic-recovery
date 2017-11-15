package com.acmerobotics.relicrecovery.drive;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.loops.Looper;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.acmerobotics.relicrecovery.path.Path;
import com.qualcomm.hardware.bosch.BNO055IMU;
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

    // TODO: should these be extracted into some kind of configuration object?
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
    private int[] offsets;

    private BNO055IMU imu;
    private double headingOffset;

    private PoseEstimator poseEstimator;
    private PathFollower pathFollower;

    /* inputs are pitch and roll, respectively */
    private PIDController balanceAxialController, balanceLateralController;

    private Mode mode = Mode.OPEN_LOOP;
    private Mode lastMode;

    private double[] powers, targetPowers;
    private Vector2d lastVel = new Vector2d(0, 0);

    private Telemetry telemetry;
    private Canvas fieldOverlay;

    public MecanumDrive(HardwareMap map) {
        this(map, null);
    }

    /**
     * construct drive with default configuration names
     * @param map hardware map
     */
    public MecanumDrive(HardwareMap map, Telemetry telemetry) {
        this(map, telemetry, new String[]{"frontLeft", "rearLeft", "rearRight", "frontRight"});
    }

    /**
     * construct drive with configuration names other than the default
     * @param map hardware map
     * @param names names of the motors in the hardware mapping
     */
    public MecanumDrive(HardwareMap map, Telemetry telemetry, String[] names) {
        if (names.length != 4) {
            throw new IllegalArgumentException("must be four for motors");
        }

        this.telemetry = telemetry;
        this.fieldOverlay = RobotDashboard.getInstance().getFieldOverlay();

        imu = map.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        imu.initialize(parameters);

        resetHeading();

        powers = new double[4];
        targetPowers = new double[4];
        offsets = new int[4];
        motors = new DcMotor[4];
        for (int i = 0; i < 4; i ++) {
            motors[i] = map.dcMotor.get(names[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }

        motors[0].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[1].setDirection(DcMotorSimple.Direction.REVERSE);

        poseEstimator = new PoseEstimator(this, new Pose2d(0, 0));
        pathFollower = new PathFollower(this, telemetry, DriveConstants.HEADING_COEFFS, DriveConstants.AXIAL_COEFFS, DriveConstants.LATERAL_COEFFS);

        balanceAxialController = new PIDController(DriveConstants.BALANCE_AXIAL_COEFFS);
        balanceAxialController.setOutputBounds(-1, 1);
        balanceAxialController.setSetpoint(0);

        balanceLateralController = new PIDController(DriveConstants.BALANCE_LATERAL_COEFFS);
        balanceLateralController.setOutputBounds(-1, 1);
        balanceLateralController.setSetpoint(0);

        resetEncoders();
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
    public void setVelocity(Vector2d vel, double omega, boolean ramp) {
        if (vel.equals(lastVel)) {
            return;
        }
        internalSetVelocity(vel, omega);
        setMode(ramp ? Mode.OPEN_LOOP_RAMP : Mode.OPEN_LOOP);
    }

    public void setVelocity(Vector2d vel, double omega) {
        setVelocity(vel, omega, false);
    }

    void internalSetVelocity(Vector2d vel, double omega) {
        targetPowers[0] = vel.x() - vel.y() - K * omega;
        targetPowers[1] = vel.x() + vel.y() - K * omega;
        targetPowers[2] = vel.x() - vel.y() + K * omega;
        targetPowers[3] = vel.x() + vel.y() + K * omega;

        double max = Collections.max(Arrays.asList(1.0, Math.abs(targetPowers[0]),
                Math.abs(targetPowers[1]), Math.abs(targetPowers[2]), Math.abs(targetPowers[3])));

        for (int i = 0; i < 4; i++) {
            targetPowers[i] /= max;
        }

        lastVel = vel;
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

    public void resetEncoders() {
        for(int i = 0; i < 4; i++) {
            offsets[i] = -motors[i].getCurrentPosition();
        }
    }

    /** @return motor rotations in radians */
    public double[] getRotations() {
        double[] rotations = new double[4];
        for (int i = 0; i < 4; i++) {
            rotations[i] = getRotation(i);
        }
        return rotations;
    }

    /** @return motor rotation in radians */
    public double getRotation(int motor) {
        return ticksToRadians(motor, getPosition(motor));
    }

    /** @return motor positions in encoder ticks */
    public int[] getPositions() {
        int[] positions = new int[4];
        for (int i = 0; i < 4; i++) {
            positions[i] = getPosition(i);
        }
        return positions;
    }

    /** @return motor position in encoder ticks */
    public int getPosition(int motor) {
        return offsets[motor] + motors[motor].getCurrentPosition();
    }

    private double ticksToRadians(int motor, int ticks) {
        double ticksPerRev = motors[motor].getMotorType().getTicksPerRev();
        return 2 * Math.PI * ticks / ticksPerRev;
    }

    private double getRawHeading() {
        return imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
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
        return poseEstimator.getPose();
    }

    public void setEstimatedPose(Pose2d pose) {
        poseEstimator.setPose(pose);
    }

    public void autoBalance() {
        setMode(Mode.AUTO_BALANCE);
        balanceAxialController.reset();
        balanceLateralController.reset();
    }

    @Override
    public void onLoop(long timestamp, long dt) {
        // pose estimation
        poseEstimator.update(timestamp);

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
                if (pathFollower.update(poseEstimator.getPose(), timestamp)) {
                    revertMode();
                }
                powers = targetPowers;
                break;
            case AUTO_BALANCE:
                Orientation angularOrientation = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ);
                double pitch = angularOrientation.secondAngle;
                double roll = angularOrientation.firstAngle;

                double axialError = balanceAxialController.getError(pitch);
                double lateralError = balanceLateralController.getError(roll);

                double axialUpdate = balanceAxialController.update(axialError);
                double lateralUpdate = balanceLateralController.update(lateralError);

                internalSetVelocity(new Vector2d(axialUpdate, lateralUpdate), 0);

                powers = targetPowers;

                break;
        }

        for (int i = 0; i < 4; i++) {
            motors[i].setPower(powers[i]);
        }

        Pose2d estimatedPose = poseEstimator.getPose();

        if (telemetry != null) {
            telemetry.addData("timestamp", timestamp);
            telemetry.addData("mode", mode);

            telemetry.addData("estimatedX", estimatedPose.x());
            telemetry.addData("estimatedY", estimatedPose.y());
            telemetry.addData("heading", estimatedPose.heading());

            for (int i = 0; i < 4; i++) {
                telemetry.addData("power" + i, powers[i]);
                telemetry.addData("pos" + i, getPosition(i));
            }

            telemetry.update();
        }

        double robotRadius = 9;
        fieldOverlay.setStroke("blue");
        fieldOverlay.setStrokeWidth(4);
        fieldOverlay.strokeLine(
            estimatedPose.x() + 0.5 * robotRadius * Math.cos(estimatedPose.heading()),
            estimatedPose.y() + 0.5 * robotRadius * Math.sin(estimatedPose.heading()),
            estimatedPose.x() + robotRadius * Math.cos(estimatedPose.heading()),
            estimatedPose.y() + robotRadius * Math.sin(estimatedPose.heading()));
        fieldOverlay.strokeCircle(estimatedPose.x(), estimatedPose.y(), robotRadius);
        RobotDashboard.getInstance().drawOverlay();
    }
}
