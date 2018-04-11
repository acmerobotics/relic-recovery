package com.acmerobotics.relicrecovery.hardware;

import android.util.Log;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.CSVLoggingTelemetry;
import com.acmerobotics.library.dashboard.telemetry.DashboardTelemetry;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.util.LoggingUtil;
import com.qualcomm.hardware.lynx.LynxAnalogInputController;
import com.qualcomm.hardware.lynx.LynxDcMotorController;
import com.qualcomm.hardware.lynx.LynxDigitalChannelController;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.LynxVoltageSensor;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.DcMotorController;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DigitalChannelController;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.VoltageSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.io.File;

@Autonomous(name="ProxTest")
public class ProxTest extends LinearOpMode{

    private AnalogInput[] inputs;

    @Override
    public void runOpMode() {

        CSVLoggingTelemetry csv = new CSVLoggingTelemetry(new File(LoggingUtil.getLogRoot(this),
                "proxTest" + System.currentTimeMillis() + ".csv"));

        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry(), csv);

        inputs = new AnalogInput[4];

        LynxModule hub = hardwareMap.get(LynxModule.class, "hub");


        AnalogInput pot;
        try {
            pot = new AnalogInput(new LynxAnalogInputController(hardwareMap.appContext, hub),
                    3 );
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        AnalogInput ir;
        try {
            ir = new AnalogInput(new LynxAnalogInputController(hardwareMap.appContext, hub),
                    1 );
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        VoltageSensor voltage;
        try {
            voltage = new LynxVoltageSensor(hardwareMap.appContext, hub);
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        DigitalChannelController touch;
        try {
            touch = new LynxDigitalChannelController(hardwareMap.appContext, hub);
        } catch (Exception e) {
            e.printStackTrace();
            return;
        }

        waitForStart();

        double potZero = 0;
        boolean recording = false;

        while (!isStopRequested()) {

            if (!touch.getDigitalChannelState(1)) {
                potZero = pot.getVoltage();
                recording = true;
            }

            if (recording) {
                telemetry.addData("pot", pot.getVoltage() - potZero);
                telemetry.addData("ir", ir.getVoltage());
                telemetry.addData("voltage", voltage.getVoltage());
                telemetry.update();
            }
        }

        csv.close();

//        LynxModule hub = hardwareMap.get(LynxModule.class, "hub");
//
//        LynxAnalogInputController controller;
//
//        try {
//            controller = new LynxAnalogInputController(hardwareMap.appContext, hub);
//            for (int i = 0; i < inputs.length; i++) {
//                inputs[i] = new AnalogInput(controller, i);
//            }
//
//        }catch (Exception e) {
//            e.printStackTrace();
//        }
//
//
//
//        waitForStart();
//
//        while (!isStopRequested()) {
//            for (int i = 0; i < inputs.length; i++) {
//                telemetry.addData("" + i, inputs[i].getVoltage());
//            }
//            telemetry.update();
//        }


    }
}
