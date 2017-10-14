package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.config.Config;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by ryanbrott on 9/17/17.
 */
@Config
public class VisionConstants {
    public static final String VUFORIA_LICENSE_KEY = "AaNzdGn/////AAAAGVCiwQaxg01ft7Lw8kYMP3aE00RU5hyTkE1CNeaYi16CBF0EC/LWi50VYsSMdJITYz6jBTmG6UGJNaNXhzk1zVIggfVmGyEZFL5doU6eVaLdgLyVmJx6jLgNzSafXSLnisXnlS+YJlCaOh1pwk08tWM8Oz+Au7drZ4BkO8j1uluIkwiewRu5zDZGlbNliFfYeCRqslBEZCGxiuH/idcsD7Q055Bwj+f++zuG3x4YlIGJCHrTpVjJUWEIbdJzJVgukc/vVOz21UNpY6WoAwH5MSeh4/U6lYwMZTQb4icfk0o1EiBdOPJKHsxyVF9l00r+6Mmdf6NJcFTFLoucvPjngWisD2T/sjbtq9N+hHnKRpbK\n";
    public static final VuforiaLocalizer.Parameters VUFORIA_PARAMETERS = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);
    public static final int FRAME_QUEUE_CAPACITY = 25;

    static {
        VUFORIA_PARAMETERS.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        VUFORIA_PARAMETERS.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VUFORIA_PARAMETERS.useExtendedTracking = false;
    }

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 0;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 112, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 0;
    public static int BLUE_UPPER_HUE = 124, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    // brown HSV range
    public static int BROWN_LOWER_HUE = 0, BROWN_LOWER_SAT = 31, BROWN_LOWER_VALUE = 27;
    public static int BROWN_UPPER_HUE = 19, BROWN_UPPER_SAT = 94, BROWN_UPPER_VALUE = 104;

    // gray HSV range
    public static int GRAY_LOWER_HUE = 66, GRAY_LOWER_SAT = 3, GRAY_LOWER_VALUE = 121;
    public static int GRAY_UPPER_HUE = 126, GRAY_UPPER_SAT = 69, GRAY_UPPER_VALUE = 210;

    // binary morphology kernel sizes
    public static int OPEN_KERNEL_SIZE = 5;
    public static int CLOSE_KERNEL_SIZE = 5;

    public static double MAX_BLOB_ASPECT_RATIO = 0.5;
    public static int MIN_BLOB_SIZE = 250;
    public static double MAX_ASPECT_RATIO_ERROR = 0.2;
    public static double MIN_RECT_FILL = 0.85;

    public static final double ACTUAL_RAIL_GAP = 7.5; // in
    public static final double ACTUAL_GLYPH_SIZE = 6.0; // in
}
