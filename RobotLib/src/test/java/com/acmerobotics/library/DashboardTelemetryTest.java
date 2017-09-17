package com.acmerobotics.library;

import com.acmerobotics.library.dashboard.DashboardTelemetry;
import com.google.gson.Gson;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.junit.Test;

/**
 * @author Ryan
 */

public class DashboardTelemetryTest {
    @Test
    public void testSerialization() {
        Telemetry telemetry = new DashboardTelemetry(null);
        telemetry.addData("hello", "world");
        telemetry.addData("num", 2);
        System.out.println(new Gson().toJson(telemetry));
    }
}
