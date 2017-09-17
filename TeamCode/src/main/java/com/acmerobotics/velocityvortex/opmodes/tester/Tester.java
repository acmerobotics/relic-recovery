package com.acmerobotics.velocityvortex.opmodes.tester;

import com.acmerobotics.velocityvortex.opmodes.StickyGamepad;
import com.qualcomm.hardware.ArmableUsbDevice;
import com.qualcomm.robotcore.exception.RobotCoreException;
import com.qualcomm.robotcore.hardware.Engagable;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareDevice;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public abstract class Tester<T extends HardwareDevice> {

    private boolean enabled;
    protected T device;
    protected String hardwareName;

    public Tester(String name, T device) {
        this.hardwareName = name;
        this.device = device;
        this.enabled = true;
    }

    public static int cycleForward(int i, int min, int max) {
        if (i == max) {
            return min;
        } else {
            return i + 1;
        }
    }

    public static int cycleBackward(int i, int min, int max) {
        if (i == min) {
            return max;
        } else {
            return i - 1;
        }
    }

    public String getName() {
        return hardwareName;
    }

    public boolean isEnabled() {
        return enabled;
    }

    public void enable() {
        if (isEnabled()) return;
        if (device instanceof Engagable) {
            Engagable engagable = (Engagable) device;
            engagable.engage();
            enabled = true;
        } else if (device instanceof ArmableUsbDevice) {
            ArmableUsbDevice armable = (ArmableUsbDevice) device;
            try {
                armable.arm();
                enabled = true;
            } catch (RobotCoreException e) {
                RobotLog.e(e.getMessage());
            } catch (InterruptedException e) {
                RobotLog.e(e.getMessage());
            }
        }
    }

    public void disable() {
        if (!isEnabled()) return;
        if (device instanceof Engagable) {
            Engagable engagable = (Engagable) device;
            engagable.disengage();
            enabled = false;
        } else if (device instanceof ArmableUsbDevice) {
            ArmableUsbDevice armable = (ArmableUsbDevice) device;
            try {
                armable.disarm();
                enabled = false;
            } catch (RobotCoreException e) {
                RobotLog.e(e.getMessage());
            } catch (InterruptedException e) {
                RobotLog.e(e.getMessage());
            }
        }
    }

    public abstract String getType();

    public abstract String getId();

    public abstract void loop(Gamepad gamepad, StickyGamepad stickyGamepad, Telemetry telemetry);
}
