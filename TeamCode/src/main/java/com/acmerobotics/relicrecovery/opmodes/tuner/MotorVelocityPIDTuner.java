package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.message.Message;
import com.acmerobotics.library.dashboard.message.MessageType;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.library.hardware.CachingDcMotor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import java.util.HashMap;
import java.util.Map;

@Autonomous
@Config
public class MotorVelocityPIDTuner extends LinearOpMode {
    public static PIDCoefficients MOTOR_PID = new PIDCoefficients();
    public static double WAVE_MAX_POWER = 0.5;
    public static double WAVE_PERIOD = 10;
    public static double MIN_POWER = 0.1;
    public static WaveType WAVE_TYPE = WaveType.SQUARE;

    public enum WaveType {
        SQUARE,
        SAWTOOTH,
        REVERSE_SAWTOOTH,
        TRIANGLE
    }

    public static final Map<WaveType, Func<Double>> WAVE_FUNCTIONS = new HashMap<>();
    static {
        WAVE_FUNCTIONS.put(WaveType.SQUARE, MotorVelocityPIDTuner::square);
        WAVE_FUNCTIONS.put(WaveType.SAWTOOTH, MotorVelocityPIDTuner::sawtooth);
        WAVE_FUNCTIONS.put(WaveType.REVERSE_SAWTOOTH, MotorVelocityPIDTuner::reverseSawtooth);
        WAVE_FUNCTIONS.put(WaveType.TRIANGLE, MotorVelocityPIDTuner::triangle);
    }

    public static double square() {
        return ((TimestampedData.getCurrentTime() % WAVE_PERIOD) < WAVE_PERIOD / 2) ? 0 : WAVE_MAX_POWER;
    }

    public static double sawtooth() {
        return (TimestampedData.getCurrentTime() % WAVE_PERIOD) / WAVE_PERIOD * WAVE_MAX_POWER;
    }

    public static double reverseSawtooth() {
        return (1 - (TimestampedData.getCurrentTime() % WAVE_PERIOD) / WAVE_PERIOD) * WAVE_MAX_POWER;
    }

    public static double triangle() {
        double x = TimestampedData.getCurrentTime() % WAVE_PERIOD / WAVE_PERIOD;
        if (x < 0.5) {
            return 2 * x * WAVE_MAX_POWER;
        } else {
            return 2 * (1 - x) * WAVE_MAX_POWER;
        }
    }

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx motor = hardwareMap.get(DcMotorEx.class, "motor");
        motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        PIDCoefficients currentCoeffs = motor.getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR_PID.p = currentCoeffs.p;
        MOTOR_PID.i = currentCoeffs.i;
        MOTOR_PID.d = currentCoeffs.d;
        RobotLog.i("Initial motor PID coefficients: p=" + MOTOR_PID.p + ",i=" + MOTOR_PID.i + ",d=" + MOTOR_PID.d);

        DcMotor cachingMotor = new CachingDcMotor(motor);

        RobotDashboard dashboard = RobotDashboard.getInstance();
        dashboard.sendAll(new Message(MessageType.RECEIVE_CONFIG, dashboard.getConfigJson()));
        Telemetry telemetry = dashboard.getTelemetry();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        MotorConfigurationType motorType = motor.getMotorType();
        double maxRpm = motorType.getAchieveableMaxRPMFraction();

        while (opModeIsActive()) {
            // update the coefficients if necessary
            if (currentCoeffs.p != MOTOR_PID.p || currentCoeffs.i != MOTOR_PID.i || currentCoeffs.d != MOTOR_PID.d) {
                RobotLog.i("Updated motor PID coefficients: p=" + MOTOR_PID.p + ",i=" + MOTOR_PID.i + ",d=" + MOTOR_PID.d);
                currentCoeffs.p = MOTOR_PID.p;
                currentCoeffs.i = MOTOR_PID.i;
                currentCoeffs.d = MOTOR_PID.d;
                motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentCoeffs);
            }
            // update the motor power
            double targetPower = WAVE_FUNCTIONS.get(WAVE_TYPE).value();
            if (Math.abs(targetPower) < MIN_POWER) {
                targetPower = 0;
            }
            cachingMotor.setPower(targetPower);
            // update telemetry
            double velocity = motor.getVelocity(AngleUnit.RADIANS);
            double targetVelocity = 33 * targetPower * maxRpm;
            telemetry.addData("velocity", velocity);
            telemetry.addData("targetVelocity", targetVelocity);
            telemetry.update();
        }
    }
}
