package com.acmerobotics.relicrecovery.subsystems;

import com.acmerobotics.relicrecovery.loops.PriorityScheduler;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 1/9/18.
 */

public class PhoneSwivel {
    private PriorityScheduler scheduler;

    private Servo servo;

    private boolean pointedAtJewel;

    public PhoneSwivel(HardwareMap map, PriorityScheduler scheduler) {
        this.scheduler = scheduler;

        servo = map.servo.get("phoneSwivel");

        pointAtJewel();
    }

    public void pointAtJewel() {
        if (!pointedAtJewel) {
            scheduler.add(() -> servo.setPosition(0), "swivel: servo set pos", PriorityScheduler.HIGH_PRIORITY);
            pointedAtJewel = true;
        }
    }

    public void pointAtCryptobox() {
        if (pointedAtJewel) {
            scheduler.add(() -> servo.setPosition(1), "swivel: servo set pos", PriorityScheduler.HIGH_PRIORITY);
            pointedAtJewel = false;
        }
    }
}
