package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsI2cColorSensor;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDcMotorController;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbDeviceInterfaceModule;
import com.qualcomm.hardware.modernrobotics.ModernRoboticsUsbServoController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.UltrasonicSensor;

import java.util.ArrayList;
import java.util.List;
import java.util.Set;

@TeleOp(name = "Tester")
public class TesterOpMode extends OpMode {

    public enum Mode {
        MAIN,
        DETAIL
    }

    private List<TesterBinding> bindings;
    private List<Tester> testers;
    private int testerIndex;
    private Mode mode;
    private StickyGamepad stickyGamepad1;
    private Set<String> names;
    private Tester currentTester;

    @Override
    public void init() {
        bindings = new ArrayList<>();
        makeBindings();

        stickyGamepad1 = new StickyGamepad(gamepad1);

        testers = new ArrayList<>();
        for (HardwareDevice device : hardwareMap.getAll(HardwareDevice.class)) {
            names = hardwareMap.getNamesOf(device);
            String name = names.size() > 0 ? names.iterator().next() : "";
            for (TesterBinding binding : bindings) {
                if (binding.matches(name, device)) {
                    if (binding.isValid())
                        testers.add(binding.newTester(name, device));
                    break;
                }
            }
        }

        mode = Mode.MAIN;
        testerIndex = 0;
    }

    public <T extends HardwareDevice> void bind(Class<T> deviceClass, Class<? extends Tester<T>> testerClass) {
        bind("", deviceClass, testerClass);
    }

    public <T extends HardwareDevice> void bind(String nameRegex, Class<T> deviceClass, Class<? extends Tester<T>> testerClass) {
        bindings.add(new TesterBinding(nameRegex, deviceClass, testerClass));
    }

    public void makeBindings() {
        bind(ModernRoboticsUsbDcMotorController.class, MRMotorControllerTester.class);
        bind(ModernRoboticsUsbServoController.class, MRServoControllerTester.class);
        bind(ModernRoboticsUsbDeviceInterfaceModule.class, MRDeviceInterfaceModuleTester.class);

        bind(ModernRoboticsI2cColorSensor.class, MRColorSensorTester.class);
        bind(DistanceSensor.class, DistanceSensorTester.class);
        bind(UltrasonicSensor.class, UltrasonicSensorTester.class);

        bind(DcMotor.class, null);
        bind(Servo.class, null);

        bind(HardwareDevice.class, DefaultTester.class);
    }

    @Override
    public void loop() {
        stickyGamepad1.update();
        switch (mode) {
            case MAIN:
                if (stickyGamepad1.dpad_down) {
                    testerIndex = Tester.cycleForward(testerIndex, 0, testers.size() - 1);
                }
                if (stickyGamepad1.dpad_up) {
                    testerIndex = Tester.cycleBackward(testerIndex, 0, testers.size() - 1);
                }

                currentTester = testers.get(testerIndex);

                if (stickyGamepad1.x) {
                    if (currentTester.isEnabled()) {
                        currentTester.disable();
                    } else {
                        currentTester.enable();
                    }
                }

                if (stickyGamepad1.dpad_right && currentTester.isEnabled()) {
                    mode = Mode.DETAIL;
                } else {
                    for (int i = 0; i < testers.size(); i++) {
                        Tester tester = testers.get(i);
                        String s = tester.getName() + " (" + tester.getType() + ")";
                        if (i == testerIndex) {
                            telemetry.addData("[>]", s);
                        } else if (!tester.isEnabled()) {
                            telemetry.addData("[X]", s);
                        } else {
                            telemetry.addData("[ ]", s);
                        }
                    }
                }
                break;
            case DETAIL:
                if (stickyGamepad1.dpad_left) {
                    mode = Mode.MAIN;
                }

                telemetry.addData("name", currentTester.getName());
                telemetry.addData("type", currentTester.getType());
                if (currentTester.getId().length() > 0)
                    telemetry.addData("id", currentTester.getId());
                currentTester.loop(gamepad1, stickyGamepad1, telemetry);
        }
    }

}
