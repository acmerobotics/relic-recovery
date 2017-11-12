package com.acmerobotics.relicrecovery.opmodes;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VisionConstants;
import com.acmerobotics.velocityvortex.drive.PIDController;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.PIDCoefficients;

import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;

/**
 * Created by ryanbrott on 10/26/17.
 */

@Config
@TeleOp(name = "Vision Alignment Test")
public class VisionAlignmentTest extends LinearOpMode {
    public static final long LOOP_MS = 20;

    private RobotDashboard dashboard;

    private VisionCamera camera;
    private CryptoboxTracker cryptoboxTracker;

    private MecanumDrive drive;

    private BNO055IMU imu;

    public static PIDCoefficients headingCoeffs = new PIDCoefficients(-0.0005, -0.0001, -0.0001);
    public static PIDCoefficients distanceCoeffs = new PIDCoefficients(0.05, 0, 0);
    public static PIDCoefficients offsetCoeffs = new PIDCoefficients(-0.1, 0, 0);

    public static double targetHeading = 0;
    public static double targetDistance = 35;
    public static double targetOffset = 0;

    private PIDController headingController, distanceController, offsetController;

    private double offset = targetOffset, distance = targetDistance;

    @Override
    public void runOpMode() throws InterruptedException {
        dashboard = RobotDashboard.getInstance();
        telemetry = new MultipleTelemetry(telemetry, dashboard.getTelemetry());

        camera = new VisionCamera(hardwareMap.appContext);
        cryptoboxTracker = new CryptoboxTracker(false);
        camera.addTracker(new FpsTracker());
        camera.addTracker(cryptoboxTracker);
        camera.initialize(VisionConstants.VUFORIA_PARAMETERS);

        imu = hardwareMap.get(BNO055IMU.class, "imu");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        imu.initialize(parameters);

        drive = new MecanumDrive(hardwareMap);

        headingController = new PIDController(headingCoeffs);
        headingController.setInputBounds(0, 360);
        headingController.setOutputBounds(-1, 1);
        distanceController = new PIDController(distanceCoeffs);
        distanceController.setOutputBounds(-1, 1);
        offsetController = new PIDController(offsetCoeffs);
        offsetController.setOutputBounds(-1, 1);

        waitForStart();

        while (opModeIsActive()) {
            long loopStart = System.currentTimeMillis();

            // update heading
            double heading = imu.getAngularOrientation().toAxesOrder(AxesOrder.XYZ).thirdAngle;
            double headingError = headingController.getError(heading, targetHeading);
            double headingUpdate = headingController.update(headingError);

            // update distance and offset
            CryptoboxTracker.CryptoboxResult result = cryptoboxTracker.getLatestResult();
            if (result.rails.size() == 4) {
                distance = result.distance;
                offset = result.offsetX;
            }
            double distanceError = distanceController.getError(distance, targetDistance);
            double offsetError = offsetController.getError(offset, targetOffset);
            double distanceUpdate = distanceController.update(distanceError);
            double offsetUpdate = offsetController.update(offsetError);

            // update drive
            drive.setVelocity(new Vector2d(offsetUpdate, distanceUpdate), headingUpdate);

            // telemetry
            telemetry.addData("heading", heading);
            telemetry.addData("headingError", headingError);
            telemetry.addData("headingUpdate", headingUpdate);

            telemetry.addData("distance", distance);
            telemetry.addData("distanceError", distanceError);
            telemetry.addData("distanceUpdate", distanceUpdate);

            telemetry.addData("offset", offset);
            telemetry.addData("offsetError", offsetError);
            telemetry.addData("offsetUpdate", offsetUpdate);

            telemetry.update();

            // consistent timing
            while ((System.currentTimeMillis() - loopStart) < LOOP_MS) {
                Thread.sleep(0, 250);
            }
        }

        camera.close();
    }
}
