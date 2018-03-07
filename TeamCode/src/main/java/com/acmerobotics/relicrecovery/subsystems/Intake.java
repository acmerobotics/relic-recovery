package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.telemetry.TelemetryEx;
import com.acmerobotics.relicrecovery.hardware.CachingDcMotor;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Config
public class Intake extends Subsystem {
    public static double GLYPH_PRESENCE_THRESHOLD = 2.5; // in (theoretically, might actually be cm)

    public enum Mode {
        AUTO,
        MANUAL
    }

    private Mode mode = Mode.MANUAL;

    private DcMotor leftIntake, rightIntake;
    private double leftIntakePower, rightIntakePower;

    private LynxI2cColorRangeSensor frontColorDistance, rearColorDistance;

    private TelemetryEx telemetry;
    private TelemetryData telemetryData;

    public class TelemetryData {
        public Mode intakeMode;
        public double leftIntakePower;
        public double rightIntakePower;

        public double intakeFrontDistance;
        public double intakeRearDistance;
        public boolean intakeHasFrontGlyph;
        public boolean intakeHasRearGlyph;
        public int intakeGlyphCount;
    }

    public Intake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = new TelemetryEx(telemetry);
        this.telemetryData = new TelemetryData();

        leftIntake = new CachingDcMotor(map.dcMotor.get("intakeLeft"));
        rightIntake = new CachingDcMotor(map.dcMotor.get("intakeRight"));

        leftIntake.setDirection(DcMotorSimple.Direction.REVERSE);
        rightIntake.setDirection(DcMotorSimple.Direction.REVERSE);

        frontColorDistance = map.get(LynxI2cColorRangeSensor.class, "frontColorDistance");
        rearColorDistance = map.get(LynxI2cColorRangeSensor.class, "rearColorDistance");
    }

    public Mode getMode() {
        return mode;
    }

    public void setIntakePower(double intakePower) {
        setIntakePower(intakePower, intakePower);
    }

    public void setIntakePower(double leftIntakePower, double rightIntakePower) {
        this.leftIntakePower = leftIntakePower;
        this.rightIntakePower = rightIntakePower;
        mode = Mode.MANUAL;
    }

    public void autoIntake() {
        mode = Mode.AUTO;
    }

    @Override
    public void update() {
        telemetryData.intakeMode = mode;
        telemetryData.leftIntakePower = leftIntakePower;
        telemetryData.rightIntakePower = rightIntakePower;

        switch (mode) {
            case AUTO:
                double frontDistance = frontColorDistance.getDistance(DistanceUnit.INCH);
                double rearDistance = rearColorDistance.getDistance(DistanceUnit.INCH);

                frontDistance = Double.isNaN(frontDistance) ? 20 : frontDistance;
                rearDistance = Double.isNaN(rearDistance) ? 20 : rearDistance;

                boolean hasFrontGlyph = frontDistance <= GLYPH_PRESENCE_THRESHOLD;
                boolean hasRearGlyph = rearDistance <= GLYPH_PRESENCE_THRESHOLD;
                int glyphCount = (hasFrontGlyph ? 1 : 0) + (hasRearGlyph ? 1 : 0);

                telemetryData.intakeFrontDistance = frontDistance;
                telemetryData.intakeRearDistance = rearDistance;
                telemetryData.intakeHasFrontGlyph = hasFrontGlyph;
                telemetryData.intakeHasRearGlyph = hasRearGlyph;
                telemetryData.intakeGlyphCount = glyphCount;

                if (glyphCount < 2) {
                    leftIntakePower = 1;
                    rightIntakePower = 1;
                } else {
                    leftIntakePower = 0;
                    rightIntakePower = 0;
                    mode = Mode.MANUAL;
                }

                break;
            case MANUAL:
                break;
        }

        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);

        telemetry.addDataObject(telemetryData);
    }
}
