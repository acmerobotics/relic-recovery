package com.acmerobotics.relicrecovery.util;

import android.os.Environment;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;

/**
 * @author Ryan
 */

public class LoggingUtil {
    public static final String BASE_DIRNAME = "ACME";

    public static File getImageDir(OpMode opMode) {
        String dirName = opMode.getClass().getSimpleName() + "-images-" + System.currentTimeMillis();
        File dir = new File(Environment.getExternalStorageDirectory(), new File(BASE_DIRNAME, dirName).getPath());
        dir.mkdirs();
        return dir;
    }
}
