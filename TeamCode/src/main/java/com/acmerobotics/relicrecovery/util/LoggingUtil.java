package com.acmerobotics.relicrecovery.util;

import android.os.Environment;

import com.acmerobotics.library.configuration.MatchType;
import com.acmerobotics.library.configuration.OpModeConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;

import java.io.File;

/**
 * @author Ryan
 */

public class LoggingUtil {
    public static File getLogRoot(OpMode opMode) {
        String dirName = opMode.hardwareMap.appContext.getResources().getString(R.string.log_root);
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        dir.mkdirs();
        return dir;
    }

    public static File getLogFile(OpMode opMode, OpModeConfiguration configuration) {
        MatchType matchType = configuration.getMatchType();
        String filenameSuffix;
        if (matchType == MatchType.PRACTICE) {
            filenameSuffix = matchType + "-" + System.currentTimeMillis();
        } else {
            filenameSuffix = matchType + "-" + configuration.getMatchNumber();
        }
        String filename = opMode.getClass().getSimpleName() + "-" + filenameSuffix + ".csv";
        return new File(getLogRoot(opMode), filename);
    }

    public static File getImageDir(OpMode opMode) {
        String dirName = opMode.getClass().getSimpleName() + "-images-" + System.currentTimeMillis();
        File dir = new File(Environment.getExternalStorageDirectory(), new File(getLogRoot(opMode), dirName).getPath());
        dir.mkdirs();
        return dir;
    }
}
