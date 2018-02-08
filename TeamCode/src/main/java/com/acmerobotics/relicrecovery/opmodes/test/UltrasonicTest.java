package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.relicrecovery.hardware.MaxSonarEZ1UltrasonicSensor;
import com.acmerobotics.relicrecovery.subsystems.JewelSlapper;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class UltrasonicTest extends OpMode {
    public static final double[] SMOOTHING_RATIOS = {1, 0.5, 0.25, 0.15, 0.1};

    private Robot robot;
    private MaxSonarEZ1UltrasonicSensor ultrasonicSensor;
    private ExponentialSmoother[] smoothers;

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());
        ultrasonicSensor = new MaxSonarEZ1UltrasonicSensor(hardwareMap.analogInput.get("ultrasonic"));
        smoothers = new ExponentialSmoother[SMOOTHING_RATIOS.length];
        for (int i = 0; i < smoothers.length; i++) {
            smoothers[i] = new ExponentialSmoother(SMOOTHING_RATIOS[i]);
        }
        robot = new Robot(this);
        robot.jewelSlapper.setArmPosition(JewelSlapper.ArmPosition.HALFWAY);
        robot.start();
    }

    @Override
    public void loop() {
        double distance = ultrasonicSensor.getDistance(DistanceUnit.INCH);
        for (int i = 0; i < smoothers.length; i++) {
            telemetry.addData("distance" + i, smoothers[i].update(distance));
        }
        try {
            Thread.sleep(50);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
    }
}
