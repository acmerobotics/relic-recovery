package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.library.hardware.MaxSonarEZ1UltrasonicSensor;
import com.acmerobotics.relicrecovery.localization.UltrasonicLocalizer;
import com.acmerobotics.relicrecovery.subsystems.Robot;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

//@Disabled
@TeleOp
public class UltrasonicTest extends OpMode {
    public static final double[] SMOOTHING_RATIOS = {1, 0.5, 0.25, 0.15, 0.1};

    private Robot robot;
    private MaxSonarEZ1UltrasonicSensor ultrasonicSensor;
    private ExponentialSmoother[] smoothers;

    @Override
    public void init() {
        ultrasonicSensor = new MaxSonarEZ1UltrasonicSensor(hardwareMap.analogInput.get("ultrasonicSensor"));
        smoothers = new ExponentialSmoother[SMOOTHING_RATIOS.length];
        for (int i = 0; i < smoothers.length; i++) {
            smoothers[i] = new ExponentialSmoother(SMOOTHING_RATIOS[i]);
        }
        robot = new Robot(this);
        robot.addListener(() -> {
            double distance = UltrasonicLocalizer.COEFFICIENT_A * ultrasonicSensor.getDistance(DistanceUnit.INCH) + UltrasonicLocalizer.COEFFICIENT_B;
            for (int i = 0; i < smoothers.length; i++) {
                robot.dashboard.getTelemetry().addData("distance" + i, smoothers[i].update(distance));
            }
        });
        robot.start();
    }

    @Override
    public void start() {
        robot.drive.extendUltrasonicSwivel();
    }

    @Override
    public void loop() {

    }
}
