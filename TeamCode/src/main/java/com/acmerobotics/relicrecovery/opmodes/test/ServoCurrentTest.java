package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.util.ExponentialSmoother;
import com.acmerobotics.library.hardware.AdafruitINA219CurrentSensor;
import com.acmerobotics.library.hardware.CurrentSensor;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
@Config
public class ServoCurrentTest extends OpMode {
    public static double POSITION = 0.5;

    private RobotDashboard dashboard;
    private Servo servo;
    private CurrentSensor currentSensor;
    private ExponentialSmoother [] smoothers;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        servo = hardwareMap.servo.get("servo");
        currentSensor = new AdafruitINA219CurrentSensor(hardwareMap.analogInput.get("currentSensor"));
        smoothers = new ExponentialSmoother[4];
        for (int i = 0; i < smoothers.length; i++) {
            smoothers[i] = new ExponentialSmoother(Math.pow(10, -i - 1));
        }
    }

    @Override
    public void loop() {
        servo.setPosition(POSITION);
        double current = currentSensor.getCurrent();
        dashboard.getTelemetry().addData("current", current);
        dashboard.getTelemetry().update();
        for (int i = 0; i < smoothers.length; i++) {
            dashboard.getTelemetry().addData("smoothed" + i, smoothers[i].update(current));
        }
    }
}
