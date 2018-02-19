package com.acmerobotics.library.util;

import android.content.Context;
import android.os.Environment;

import com.acmerobotics.library.R;
import com.acmerobotics.relicrecovery.configuration.MatchType;
import com.acmerobotics.relicrecovery.configuration.OpModeConfiguration;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import java.io.File;

public class LoggingUtil {
    public static File getLogRoot(Context context) {
        String dirName = context.getResources().getString(R.string.log_root);
        File dir = new File(Environment.getExternalStorageDirectory(), dirName);
        dir.mkdirs();
        return dir;
    }

    public static File getLogRoot(OpMode opMode) {
        return getLogRoot(opMode.hardwareMap.appContext);
    }

    private static void removeRecursive(File file) {
        if (file.isDirectory()) {
            for (File childFile : file.listFiles()) {
                removeRecursive(childFile);
            }
        }
        file.delete();
    }

    public static void clearLogs(Context context) {
        removeRecursive(getLogRoot(context));
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
