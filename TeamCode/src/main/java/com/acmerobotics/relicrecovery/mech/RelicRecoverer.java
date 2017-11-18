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
            public double diff() {return RelicRecovererConstants.UP_OFFSET;}
        },
        CLOSED {
            public double diff() {return 0;}
        },
        OPEN {
            public double diff() {return RelicRecovererConstants.OPEN_OFFSET;}
        };
        public abstract double diff();
    }

    private double extension;
    private double offset;
    private double extendOffset, retractOffset;
    private PIDController extendController, offsetController;
    private DcMotor motorExtend, motorRetract;

    private Telemetry telemetry;

    public RelicRecoverer(HardwareMap map, Telemetry telemetry) {
        motorExtend = map.dcMotor.get("relicExtend");
        motorRetract = map.dcMotor.get("relicRetract");
        extension = 0;
        extendOffset = -motorExtend.getCurrentPosition();
        retractOffset = -motorRetract.getCurrentPosition();
        extendController = new PIDController(RelicRecovererConstants.EXTENSION_COEFFICIENTS);
        offsetController = new PIDController(RelicRecovererConstants.OFFSET_COEFFICIENTS);
        extendController.setInputBounds(-RelicRecovererConstants.MAX_EXTENSION_CORRECTION, RelicRecovererConstants.MAX_EXTENSION_CORRECTION);
        offsetController.setInputBounds(-1.0 + RelicRecovererConstants.MAX_EXTENSION_CORRECTION, 1.0 - RelicRecovererConstants.MAX_EXTENSION_CORRECTION);
    }

    public void setPosition(Position position) {
        this.offset = position.diff();
    }

    public void setExtension(double extension) {
        this.extension = extension;

    }

    public void setOffset(double offset) {
        this.offset = Math.max(0, Math.min(offset, RelicRecovererConstants.MAX_EXTENSION));
    }

    public double getExtension() {
        return motorRetract.getCurrentPosition();
    }

    public double getExtensionTarget() {
        return extension;
    }

    public double getOffset() {
        return motorExtend.getCurrentPosition() - motorRetract.getCurrentPosition();
    }

    public double getOffsetTarget() {
        return offset;
    }

    public double getExtendError() {
        return getExtension() - extension;
    }

    public double getOffsetError() {
        return getOffset() - offset;
    }

    @Override
    public void onLoop(long timestamp, long dt) {
        double extendCorrection = extendController.update(getExtendError());
        double offsetCorrection = offsetController.update(getOffsetError());
        motorRetract.setPower(extendCorrection);
        motorExtend.setPower(extendCorrection + offsetCorrection);

        if (telemetry != null) {
            telemetry.addData("relicExtendError", getExtendError());
            telemetry.addData("relicOffsetError", getOffsetError());
            telemetry.addData("relicExtend", getExtension());
            telemetry.addData("relicOffset", getOffset());
        }
    }
}
