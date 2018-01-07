package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.hardware.lynx.LynxEmbeddedIMU;
import com.qualcomm.hardware.lynx.LynxI2cDeviceSynchV1;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.I2cDeviceSynch;

import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

/**
 * Created by ryanbrott on 1/6/18.
 */

public class LynxEmbeddedIMUFactory {

    public static LynxEmbeddedIMU createLynxEmbeddedIMU(LynxModule module) {
        I2cDeviceSynch imuI2cDeviceSynch = new BetterI2cDeviceSynchImplOnSimple(
                new LynxI2cDeviceSynchV1(AppUtil.getDefContext(), module, 0), true);
        imuI2cDeviceSynch.setUserConfiguredName("embedded imu for " + module.getDeviceName());
        return new LynxEmbeddedIMU(imuI2cDeviceSynch);
    }

}
