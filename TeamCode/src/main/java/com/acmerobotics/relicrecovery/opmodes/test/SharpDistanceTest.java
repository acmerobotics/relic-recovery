package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.hardware.SharpGP2Y0A51SK0FProximitySensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SharpDistanceTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());
        AnalogInput analogInput = hardwareMap.analogInput.get("proximitySensor");
        SharpGP2Y0A51SK0FProximitySensor proximitySensor = new SharpGP2Y0A51SK0FProximitySensor(analogInput);

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("voltage", analogInput.getVoltage());
            telemetry.addData("distance", proximitySensor.getDistance(SharpGP2Y0A51SK0FProximitySensor.Surface.CRYPTO, DistanceUnit.CM));
            telemetry.update();
        }
    }
}
