package com.acmerobotics.relicrecovery.opmodes.tuner;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.message.Message;
import com.acmerobotics.library.dashboard.message.MessageType;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataCommand;
import com.qualcomm.hardware.lynx.commands.core.LynxGetBulkInputDataResponse;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.configuration.MotorConfigurationType;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Autonomous
@Config
public class DriveVelocityPIDTuner extends LinearOpMode {
    public static PIDCoefficients MOTOR_PID = new PIDCoefficients();
    public static double DISTANCE = 72;
    public static double PERIOD = 30;

    @Override
    public void runOpMode() throws InterruptedException {
        DcMotorEx[] motors = new DcMotorEx[4];
        for (int i = 0; i < 4; i ++) {
            motors[i] = hardwareMap.get(DcMotorEx.class, MecanumDrive.MOTOR_NAMES[i]);
            motors[i].setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            motors[i].setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        }
        motors[2].setDirection(DcMotorSimple.Direction.REVERSE);
        motors[3].setDirection(DcMotorSimple.Direction.REVERSE);

        PIDCoefficients currentCoeffs = motors[0].getPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER);
        MOTOR_PID.p = currentCoeffs.p;
        MOTOR_PID.i = currentCoeffs.i;
        MOTOR_PID.d = currentCoeffs.d;
        RobotLog.i("Initial motor PID coefficients: p=" + MOTOR_PID.p + ",i=" + MOTOR_PID.i + ",d=" + MOTOR_PID.d);

        LynxModule frontHub = hardwareMap.get(LynxModule.class, "frontHub");

        RobotDashboard dashboard = RobotDashboard.getInstance();
        dashboard.sendAll(new Message(MessageType.RECEIVE_CONFIG, dashboard.getConfigJson()));
        Telemetry telemetry = dashboard.getTelemetry();

        telemetry.log().add("Ready!");
        telemetry.update();
        telemetry.clearAll();

        waitForStart();

        if (isStopRequested()) return;

        MotorConfigurationType motorType = motors[0].getMotorType();
        double maxRpm = motorType.getAchieveableMaxRPMFraction();

        double startTimestamp = TimestampedData.getCurrentTime();

        while (opModeIsActive()) {
            // update the coefficients if necessary
            if (currentCoeffs.p != MOTOR_PID.p || currentCoeffs.i != MOTOR_PID.i || currentCoeffs.d != MOTOR_PID.d) {
                RobotLog.i("Updated motor PID coefficients: p=" + MOTOR_PID.p + ",i=" + MOTOR_PID.i + ",d=" + MOTOR_PID.d);
                currentCoeffs.p = MOTOR_PID.p;
                currentCoeffs.i = MOTOR_PID.i;
                currentCoeffs.d = MOTOR_PID.d;
                for (DcMotorEx motor : motors) {
                    motor.setPIDCoefficients(DcMotor.RunMode.RUN_USING_ENCODER, currentCoeffs);
                }
            }
            // calculate the motor power
            double accelTime = PERIOD / 4;
            double acceleration = DISTANCE / (accelTime * accelTime);
            double periodTime = (TimestampedData.getCurrentTime() - startTimestamp) % PERIOD;
            double targetVelocity;
            if (periodTime < PERIOD / 4) {
                targetVelocity = acceleration * periodTime;
            } else if (periodTime < PERIOD / 2) {
                targetVelocity = acceleration * (PERIOD / 2 - periodTime);
            } else if (periodTime < 3 * PERIOD / 4) {
                targetVelocity = -acceleration * (periodTime - PERIOD / 2);
            } else {
                targetVelocity = -acceleration * (PERIOD - periodTime);
            }
            double rpm = 60 * targetVelocity / (2 * Math.PI * MecanumDrive.RADIUS);
            double power = rpm / maxRpm;
            for (DcMotorEx motor : motors) {
                motor.setPower(power);
            }
            // retrieve the motor velocities
            double[] velocities = new double[4];
            try {
                LynxGetBulkInputDataCommand command = new LynxGetBulkInputDataCommand(frontHub);
                LynxGetBulkInputDataResponse response = command.sendReceive();
                for (int i = 0; i < 4; i++) {
                    velocities[i] = response.getVelocity(i) * (2 * Math.PI * MecanumDrive.RADIUS);
                }
                velocities[2] = -velocities[2];
                velocities[3] = -velocities[3];
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
            } catch (Exception e) {
                RobotLog.logStackTrace(e);
            }
            // update telemetry
            telemetry.addData("targetVelocity", targetVelocity);
            for (int i = 0; i < 4; i++) {
                telemetry.addData("velocity" + i, velocities[i]);
            }
            telemetry.update();
        }
    }
}
