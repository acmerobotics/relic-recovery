package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.hardware.SharpGP2Y0A51SK0FProximitySensor;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
public class SharpDistanceTest extends LinearOpMode {
    @Override
    public void runOpMode() {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());
        SharpGP2Y0A51SK0FProximitySensor proximitySensor = new SharpGP2Y0A51SK0FProximitySensor(hardwareMap.analogInput.get("proximitySensor"));

        waitForStart();

        while (opModeIsActive()) {
            telemetry.addData("redDistance", proximitySensor.getDistance(SharpGP2Y0A51SK0FProximitySensor.Surface.RED_CRYPTO, DistanceUnit.CM));
            telemetry.addData("blueDistance", proximitySensor.getDistance(SharpGP2Y0A51SK0FProximitySensor.Surface.BLUE_CRYPTO, DistanceUnit.CM));
            telemetry.update();
        }
    }
}
