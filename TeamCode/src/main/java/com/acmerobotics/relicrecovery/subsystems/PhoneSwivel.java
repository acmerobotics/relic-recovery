package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/9/18.
 */

public class PhoneSwivel {
    private Servo servo;

    private boolean pointedAtJewel;

    public PhoneSwivel(HardwareMap map) {
        servo = map.servo.get("phoneSwivel");

        pointAtJewel();
    }

    public void pointAtJewel() {
        if (!pointedAtJewel) {
            servo.setPosition(0);
            pointedAtJewel = true;
        }
    }

    public void pointAtCryptobox() {
        if (pointedAtJewel) {
            servo.setPosition(1);
            pointedAtJewel = false;
        }
    }
}
