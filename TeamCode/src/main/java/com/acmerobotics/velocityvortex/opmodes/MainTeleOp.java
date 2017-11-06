package com.acmerobotics.velocityvortex.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.velocityvortex.drive.EnhancedMecanumDrive;
import com.acmerobotics.velocityvortex.drive.MecanumDrive;
import com.acmerobotics.velocityvortex.drive.Vector2d;
import com.acmerobotics.velocityvortex.mech.BeaconPusher;
import com.acmerobotics.velocityvortex.mech.Collector;
import com.acmerobotics.velocityvortex.mech.FixedLauncher;
import com.acmerobotics.velocityvortex.sensors.LinearPot;
import com.qualcomm.hardware.adafruit.AdafruitBNO055IMU;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DeviceInterfaceModule;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "VV MainTeleOp", group = "MainTeleOp")
public class MainTeleOp extends OpMode {

    public static final int BLUE_LED_CHANNEL = 0;
    public static final int RED_LED_CHANNEL = 1;

    public static final int FLASH_MS = 150;
    public static final double MAX_LAUNCHER_POWER = 0.9;
    public static final double REDUCED_POWER = 0.6;

    private RobotDashboard dashboard;

    private MecanumDrive basicDrive;

    private FixedLauncher launcher;
    private Collector collector;
    private BeaconPusher beaconPusher;

    private StickyGamepad stickyGamepad1, stickyGamepad2;

    private AdafruitBNO055IMU imu;
    private EnhancedMecanumDrive drive;

    private DeviceInterfaceModule dim;

    private ElapsedTime flashTimer;
    private boolean shouldFlash, collectorReversed, launcherRunning;

    private boolean reducePower;

    private double launcherSpeed;

    @Override
    public void init() {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        hardwareMap.dcMotor.get("leftFront").setDirection(DcMotorSimple.Direction.REVERSE);
        hardwareMap.dcMotor.get("leftBack").setDirection(DcMotorSimple.Direction.REVERSE);

        basicDrive = new MecanumDrive(hardwareMap);

        try {
            imu = new AdafruitBNO055IMU(hardwareMap.i2cDeviceSynch.get("imu"));
            AdafruitBNO055IMU.Parameters parameters = new AdafruitBNO055IMU.Parameters();
            parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
            imu.initialize(parameters);

            drive = new EnhancedMecanumDrive(basicDrive, imu);
        } catch (Throwable t) {
            telemetry.addData("WARNING", "IMU did not initialize!");
            imu = null;
            drive = null;
        }

        launcher = new FixedLauncher(hardwareMap);

        collector = new Collector(hardwareMap);
        beaconPusher = new BeaconPusher(hardwareMap, new LinearPot(hardwareMap.analogInput.get("lp"), 200, DistanceUnit.MM));

        stickyGamepad1 = new StickyGamepad(gamepad1);
        stickyGamepad2 = new StickyGamepad(gamepad2);

        flashTimer = new ElapsedTime();

        dim = hardwareMap.deviceInterfaceModule.get("dim");
    }

    @Override
    public void loop() {
        // update gamepads
        stickyGamepad1.update();
        stickyGamepad2.update();

        //driver
        double x = -gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double omega = -gamepad1.right_stick_x;

        if (reducePower) {
            x *= REDUCED_POWER;
            y *= REDUCED_POWER;
            omega *= REDUCED_POWER;
        }

        // check the dpad
        if (gamepad1.dpad_up) y = -1;
        else if (gamepad1.dpad_down) y = 1;
        if (gamepad1.dpad_right) x = 1;
        else if (gamepad1.dpad_left) x = -1;

        // apply quadratic (square) function
        double radius = Math.pow(Math.hypot(x, y), 2);
        double theta = Math.atan2(y, x);
        omega = Math.signum(omega) * Math.pow(omega, 2);

        telemetry.addData("radius", radius);
        telemetry.addData("theta", theta);
        telemetry.addData("omega", omega);

        basicDrive.setVelocity(new Vector2d(radius * Math.cos(theta), radius * Math.sin(theta)), omega);

        if (stickyGamepad1.y) {
            reducePower = !reducePower;
        }

        if (gamepad1.left_bumper && gamepad1.right_bumper) {
            if (drive != null) drive.resetHeading();
        } else {
            //collector
            if (gamepad1.right_trigger > 0.95) {
                collector.reverse();
                collectorReversed = true;
            } else {
                if (stickyGamepad1.right_bumper) {
                    if (collector.isRunning()) {
                        collector.stop();
                    } else {
                        collector.forward();
                    }
                } else if (collectorReversed) {
                    collector.stop();
                    collectorReversed = false;
                }
            }

            if (gamepad1.left_bumper) {
                beaconPusher.extend();
            } else {
                beaconPusher.retract();
            }
            beaconPusher.update();
        }

        //launcher
        //trigger
        if (gamepad2.right_bumper) {
            launcher.triggerUp();
        } else {
            launcher.triggerDown();
        }

        //wheels
        if (stickyGamepad2.left_bumper) {
            if (launcher.isRunning()) {
                launcher.setPower(0);
                launcherRunning = false;
            } else {
                launcher.setPower(MAX_LAUNCHER_POWER);
            }
        }
        launcher.update();

        launcherSpeed = launcher.getLeftSpeed();
        if (launcherSpeed > 2.5 && launcher.isRunning()) {
            launcherRunning = true;
        }

        // launcher dim lights
        if (launcherRunning) {
            if (flashTimer.milliseconds() >= FLASH_MS) {
                shouldFlash = !shouldFlash;
                flashTimer.reset();
            }
        } else {
            shouldFlash = false;
        }

        dim.setLED(BLUE_LED_CHANNEL, shouldFlash);
        dim.setLED(RED_LED_CHANNEL, shouldFlash);

        telemetry.addData("pusher", beaconPusher.getCurrentPosition());
        telemetry.addData("heading", drive.getHeading());
        telemetry.addData("leftSpeed", launcher.getLeftSpeed());
        telemetry.addData("rightSpeed", launcher.getRightSpeed());
        telemetry.addData("launcherVoltage", launcher.getVoltage());
        telemetry.addData("pusherPosition", beaconPusher.getCurrentPosition());
    }
}
