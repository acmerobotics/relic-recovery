package com.acmerobotics.relicrecovery.opmodes;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.teamcode.R;

/**
 * Created by ryanbrott on 9/17/17.
 */

public class VisionConstants {
    public static final String VUFORIA_LICENSE_KEY = "AaNzdGn/////AAAAGVCiwQaxg01ft7Lw8kYMP3aE00RU5hyTkE1CNeaYi16CBF0EC/LWi50VYsSMdJITYz6jBTmG6UGJNaNXhzk1zVIggfVmGyEZFL5doU6eVaLdgLyVmJx6jLgNzSafXSLnisXnlS+YJlCaOh1pwk08tWM8Oz+Au7drZ4BkO8j1uluIkwiewRu5zDZGlbNliFfYeCRqslBEZCGxiuH/idcsD7Q055Bwj+f++zuG3x4YlIGJCHrTpVjJUWEIbdJzJVgukc/vVOz21UNpY6WoAwH5MSeh4/U6lYwMZTQb4icfk0o1EiBdOPJKHsxyVF9l00r+6Mmdf6NJcFTFLoucvPjngWisD2T/sjbtq9N+hHnKRpbK\n";
    public static final VuforiaLocalizer.Parameters VUFORIA_PARAMETERS = new VuforiaLocalizer.Parameters(R.id.cameraMonitorViewId);

    static {
        VUFORIA_PARAMETERS.vuforiaLicenseKey = VUFORIA_LICENSE_KEY;
        VUFORIA_PARAMETERS.cameraDirection = VuforiaLocalizer.CameraDirection.BACK;
        VUFORIA_PARAMETERS.useExtendedTracking = false;
    }
}
