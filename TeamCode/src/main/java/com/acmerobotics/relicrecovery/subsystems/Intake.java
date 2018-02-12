package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.hardware.lynx.LynxI2cColorRangeSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import java.util.LinkedHashMap;

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

    public static final String[] CONDITIONAL_TELEMETRY_KEYS = {
            "intakeHasFrontGlyph",
            "intakeHasRearGlyph",
            "intakeGlyphCount"
    };

    private Telemetry telemetry;
    private LinkedHashMap<String, Object> telemetryMap;

    public Intake(HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        telemetryMap = new LinkedHashMap<>();
        for (String key : CONDITIONAL_TELEMETRY_KEYS) {
            telemetryMap.put(key, 0);
        }

        leftIntake = map.dcMotor.get("intakeLeft");
        rightIntake = map.dcMotor.get("intakeRight");

        frontColorDistance = map.get(LynxI2cColorRangeSensor.class, "frontColorDistance");
        rearColorDistance = map.get(LynxI2cColorRangeSensor.class, "rearColorDistance");
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
        telemetry.addData("intakeMode", mode);
        telemetry.addData("leftIntakePower", leftIntakePower);
        telemetry.addData("rightIntakePower", rightIntakePower);

        switch (mode) {
            case AUTO:
                double frontDistance = frontColorDistance.getDistance(DistanceUnit.INCH);
                double rearDistance = rearColorDistance.getDistance(DistanceUnit.INCH);

                frontDistance = Double.isNaN(frontDistance) ? 20 : frontDistance;
                rearDistance = Double.isNaN(rearDistance) ? 20 : rearDistance;

                boolean hasFrontGlyph = frontDistance <= GLYPH_PRESENCE_THRESHOLD;
                boolean hasRearGlyph = rearDistance <= GLYPH_PRESENCE_THRESHOLD;
                int glyphCount = (hasFrontGlyph ? 1 : 0) + (hasRearGlyph ? 1 : 0);

                telemetry.addData("intakeFrontDistance", frontDistance);
                telemetry.addData("intakeRearDistance", rearDistance);
                telemetry.addData("intakeHasFrontGlyph", hasFrontGlyph);
                telemetry.addData("intakeHasRearGlyph", hasRearGlyph);
                telemetry.addData("intakeGlyphCount", glyphCount);

//                if (glyphCount < 2) {
//                    leftIntakePower = 1;
//                    rightIntakePower = 1;
//                } else {
//                    setIntakePower(-1, -1);
//                }

                break;
            case MANUAL:
                break;
        }

        leftIntake.setPower(leftIntakePower);
        rightIntake.setPower(rightIntakePower);
    }
}
