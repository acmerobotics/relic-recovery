package com.acmerobotics.relicrecovery.hardware;

import com.qualcomm.robotcore.hardware.I2cDeviceSynchImplOnSimple;
import com.qualcomm.robotcore.hardware.I2cDeviceSynchSimple;

/**
 * Created by ryanbrott on 1/6/18.
 */

public class BetterI2cDeviceSynchImplOnSimple extends I2cDeviceSynchImplOnSimple {
    public BetterI2cDeviceSynchImplOnSimple(I2cDeviceSynchSimple simple, boolean isSimpleOwned) {
        super(simple, isSimpleOwned);
    }

    @Override
    public void setReadWindow(ReadWindow window) {
        // intentionally do nothing
    }
}
