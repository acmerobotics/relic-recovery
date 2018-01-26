package com.acmerobotics.relicrecovery.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class PhoneSwivel extends Subsystem {
    public static final double JEWEL_POSITION = 0.15;
    public static final double CRYPTO_POSITION = 0.65;

    private Servo swivel;

    public PhoneSwivel(HardwareMap map) {
        swivel = map.servo.get("phoneSwivel");

        pointAtJewel();
    }

    public void pointAtJewel() {
        swivel.setPosition(JEWEL_POSITION);
    }

    public void pointAtCryptobox() {
        swivel.setPosition(CRYPTO_POSITION);
    }

    @Override
    public void update() {

    }
}
