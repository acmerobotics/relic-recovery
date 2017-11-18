package com.acmerobotics.relicrecovery.mech;

import com.acmerobotics.library.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Created by ryanbrott on 11/16/17.
 */

@Config
public class GlyphGripper {
    public static double GLYPH_GRIP = 0.15;
    public static double GLYPH_RELEASE = 0.4;

    private Servo glyphGripper;

    public GlyphGripper(HardwareMap hardwareMap) {
        glyphGripper = hardwareMap.servo.get("glyphGripper");
        grip();
    }

    public void grip() {
        glyphGripper.setPosition(GLYPH_GRIP);
    }

    public void release() {
        glyphGripper.setPosition(GLYPH_RELEASE);
    }
}
