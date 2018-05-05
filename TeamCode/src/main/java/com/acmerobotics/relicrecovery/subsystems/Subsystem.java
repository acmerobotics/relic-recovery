package com.acmerobotics.relicrecovery.subsystems;

import java.util.Map;

public abstract class Subsystem {
    /**
     * Run control code (e.g., read sensors and update motors) and return telemetry.
     */
    public abstract Map<String, Object> update();
}
