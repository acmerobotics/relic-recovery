package com.acmerobotics.relicrecovery.mech;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ACME Robotics on 9/21/2017.
 */

public class JewelSlapper {

    public static final double JEWEL_SLAPPER_UP = 0;
    public static final double JEWEL_SLAPPER_DOWN = 0.31;

    private Servo jewelSlapper;
    private boolean jewelSlapperActivated;

    public JewelSlapper(HardwareMap hardwareMap) {
        jewelSlapper = hardwareMap.servo.get("jewelSlapper");
        jewelSlapper.setPosition(JEWEL_SLAPPER_UP);
    }

    public void jewelSlapperUp() {
        if (!jewelSlapperActivated) {
            jewelSlapper.setPosition(JEWEL_SLAPPER_UP);
            jewelSlapperActivated = false;
        }
    }

    public void jewelSlapperDown() {
        if (jewelSlapperActivated) {
            jewelSlapper.setPosition(JEWEL_SLAPPER_DOWN);
            jewelSlapperActivated = true;
        }
    }

    public void slapperToggle() {
        if (jewelSlapperActivated) {
            jewelSlapperUp();
        }
        else {
            jewelSlapperDown();
        }
    }

    public boolean isJewelSlapperActivated() {
        return jewelSlapperActivated;
    }
    
}

