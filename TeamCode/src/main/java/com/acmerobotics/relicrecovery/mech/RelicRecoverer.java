package com.acmerobotics.relicrecovery.mech;

import com.acmerobotics.relicrecovery.loops.Loop;
import com.acmerobotics.relicrecovery.motion.PIDController;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

/**
 * @author kellyrm
 *
 */

public class RelicRecoverer implements Loop{

    public enum Position {
        //todo find the actual values
        UP {
            public double getOffset() {return RelicRecovererConstants.UP_OFFSET;}
        },
        CLOSED {
            public double getOffset() {return 0;}
        },
        OPEN {
            public double getOffset() {return RelicRecovererConstants.OPEN_OFFSET;}
        };
        public abstract double getOffset();
    }

    private Telemetry telemetry;
    private double extendSpeed;
    private double offsetTarget;
    private double offsetSpeed;
    private double extendOffset, retractOffset;
    private DcMotor motorExtend, motorRetract;
    private PIDController offsetController;

    public RelicRecoverer (HardwareMap map, Telemetry telemetry) {
        this.telemetry = telemetry;
        motorExtend = map.dcMotor.get("relicExtend");
        motorRetract = map.dcMotor.get("relicRetract");
        extendOffset = -motorExtend.getCurrentPosition();
        retractOffset = -motorRetract.getCurrentPosition();
        offsetController = new PIDController(RelicRecovererConstants.OFFSET_COEFFICIENTS);
        offsetTarget = 0;
        extendSpeed = 0;
        offsetSpeed = 0;

    }

    public double getRetractPosition() {
        return motorRetract.getCurrentPosition() + retractOffset;
    }

    public double getExtendPosition() {
        return motorExtend.getCurrentPosition() + extendOffset;
    }

    public double getOffset() {
        return getExtendPosition() - getRetractPosition();
    }

    public double getOffsetError() {
        return getOffset() - offsetTarget;
    }

    public void setOffsetSpeed(double speed) {
        offsetSpeed = speed;
    }

    public void setExtendSpeed(double speed) {
        if (getExtendPosition() >= 0 && getExtendPosition() <= RelicRecovererConstants.MAX_EXTENSION)
            extendSpeed = speed;
        else
            extendSpeed = 0;
    }

    public void setPosition(Position position) {
        offsetTarget = position.getOffset();
    }

    @Override
    public void loop(long timestamp, long dt) {

        offsetTarget += offsetSpeed * dt * RelicRecovererConstants.OFFSET_SPEEDS;
        double offsetCorrection = offsetController.update(getOffsetError());
        motorRetract.setPower(extendSpeed);
        motorExtend.setPower(extendSpeed + offsetCorrection);

        if (telemetry != null) {
            telemetry.addData("relicOffsetError", getOffsetError());
            telemetry.addData("relicExtend", getExtendPosition());
            telemetry.addData("relicRetract", getRetractPosition());
            telemetry.addData("relicOffset", getOffset());
        }
    }
}
