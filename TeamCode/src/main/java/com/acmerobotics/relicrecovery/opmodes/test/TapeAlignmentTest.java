package com.acmerobotics.relicrecovery.opmodes.test;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.telemetry.MultipleTelemetry;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.MecanumDrive;
import com.acmerobotics.relicrecovery.opmodes.TimedOpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.hardware.ColorSensor;

/**
 * @author Ryan
 */

@Autonomous(name = "Tape Alignment Test")
public class TapeAlignmentTest extends TimedOpMode {
    public static final double SPEED = 0.3;

    private enum State {
        SEARCH,
        FOLLOW_LEFT_TAPE,
        FOLLOW_RIGHT_TAPE,
        ALIGNED
    }

    private MecanumDrive drive;
    private ColorSensor leftColor, rightColor;

    private State state;

    public TapeAlignmentTest() {
        super(20);
    }

    @Override
    public void init() {
        telemetry = new MultipleTelemetry(telemetry, RobotDashboard.getInstance().getTelemetry());

        drive = new MecanumDrive(hardwareMap);
        leftColor = hardwareMap.colorSensor.get("leftColor");
        rightColor = hardwareMap.colorSensor.get("rightColor");

        drive.registerLoops(looper);

        state = State.SEARCH;
    }

    @Override
    public void loop(long timestamp, long dt) {
        telemetry.addData("state", state);

        telemetry.addData("leftRed", leftColor.red());
        telemetry.addData("leftGreen", leftColor.green());
        telemetry.addData("leftBlue", leftColor.blue());
        telemetry.addData("leftAlpha", leftColor.alpha());

        telemetry.addData("rightRed", rightColor.red());
        telemetry.addData("rightGreen", rightColor.green());
        telemetry.addData("rightBlue", rightColor.blue());
        telemetry.addData("rightAlpha", rightColor.alpha());

        switch (state) {
            case SEARCH:
                if (leftOnLine()) {
                    state = State.FOLLOW_LEFT_TAPE;
                    drive.stop();
                } else if (rightOnLine()) {
                    state = State.FOLLOW_RIGHT_TAPE;
                    drive.stop();
                } else {
                    drive.setVelocity(new Vector2d(SPEED, 0), 0);
                }
                break;
            case FOLLOW_LEFT_TAPE:
                if (rightOnLine()) {
                    state = State.ALIGNED;
                    drive.stop();
                } else {
                    drive.setVelocity(
                            leftOnLine() ? new Vector2d(SPEED, 0) : new Vector2d(0, SPEED), 0);
                }
                break;
            case FOLLOW_RIGHT_TAPE:
                if (leftOnLine()) {
                    state = State.ALIGNED;
                    drive.stop();
                } else {
                    drive.setVelocity(
                            rightOnLine() ? new Vector2d(SPEED, 0) : new Vector2d(0, -SPEED), 0);
                }
                break;
            case ALIGNED:
                break;
        }
    }

    private boolean leftOnLine() {
        return leftColor.alpha() > 500;
    }

    private boolean rightOnLine() {
        return rightColor.alpha() > 500;
    }
}
