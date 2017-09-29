package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.Config;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;

/**
 * Created by ryanbrott on 9/17/17.
 */
@Config
public class VisionConstants {
    public static final String VUFORIA_LICENSE_KEY = "AaNzdGn/////AAAAGVCiwQaxg01ft7Lw8kYMP3aE00RU5hyTkE1CNeaYi16CBF0EC/LWi50VYsSMdJITYz6jBTmG6UGJNaNXhzk1zVIggfVmGyEZFL5doU6eVaLdgLyVmJx6jLgNzSafXSLnisXnlS+YJlCaOh1pwk08tWM8Oz+Au7drZ4BkO8j1uluIkwiewRu5zDZGlbNliFfYeCRqslBEZCGxiuH/idcsD7Q055Bwj+f++zuG3x4YlIGJCHrTpVjJUWEIbdJzJVgukc/vVOz21UNpY6WoAwH5MSeh4/U6lYwMZTQb4icfk0o1EiBdOPJKHsxyVF9l00r+6Mmdf6NJcFTFLoucvPjngWisD2T/sjbtq9N+hHnKRpbK\n";
    public static final VuforiaLocalizer.Parameters VUFORIA_PARAMETERS = new VuforiaLocalizer.Parameters();
    public static final int FRAME_QUEUE_CAPACITY = 25;

    static {
        VUFORIA_PARAMETERS.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        VUFORIA_PARAMETERS.cameraDirection = VuforiaLocalizer.CameraDirection.FRONT;
        VUFORIA_PARAMETERS.useExtendedTracking = false;
    }

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 0;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 110, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 0;
    public static int BLUE_UPPER_HUE = 130, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    // binary morphology kernel sizes
    public static int OPEN_KERNEL_SIZE = 7;
    public static int CLOSE_KERNEL_SIZE = 15;

    public static int SMALL_DIMENSION = 480;
}
