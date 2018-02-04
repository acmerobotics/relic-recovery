package com.acmerobotics.relicrecovery.util;

import android.os.Environment;

import com.acmerobotics.relicrecovery.configuration.MatchType;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import org.firstinspires.ftc.teamcode.R;

import java.io.File;

public class LoggingUtil {
    public static File getLogRoot(OpMode opMode) {
        String dirName = opMode.hardwareMap.appContext.getResources().getString(R.string.log_root);
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        dir.mkdirs();
        return dir;
    }

    private static String getLogBaseName(OpMode opMode, OpModeConfiguration configuration) {
        MatchType matchType = configuration.getMatchType();
        String filenameSuffix;
        if (matchType == MatchType.PRACTICE) {
            filenameSuffix = matchType + "-" + System.currentTimeMillis();
        } else {
            filenameSuffix = matchType + "-" + configuration.getMatchNumber();
        }
        return opMode.getClass().getSimpleName() + "-" + filenameSuffix;
    }

    public static File getLogFile(OpMode opMode, OpModeConfiguration configuration) {
        return new File(getLogRoot(opMode), getLogBaseName(opMode, configuration) + ".csv");
    }

    public static File getLogDir(OpMode opMode, OpModeConfiguration configuration) {
        File dir = new File(getLogRoot(opMode), getLogBaseName(opMode, configuration));
        dir.mkdirs();
        return dir;
    }
}
