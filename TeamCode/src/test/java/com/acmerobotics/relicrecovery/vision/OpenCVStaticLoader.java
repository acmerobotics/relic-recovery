package com.acmerobotics.relicrecovery.vision;

import java.io.File;

public class OpenCVStaticLoader {
    private static boolean hasLoaded;

    public static void loadStaticLibs() {
        if (!hasLoaded) {
            File libsDir = new File("libs");
            String os = System.getProperty("os.name");
            if (os.startsWith("Windows")) {
                System.load(new File(libsDir, "opencv_java331.dll").getAbsolutePath());
            } else if (os.startsWith("Mac OS")) {
                System.load(new File(libsDir, "opencv_java331.dylib").getAbsolutePath());
            } else {
                System.err.println("Unable to load OpenCV libs on " + os);
                System.exit(-1);
            }
            hasLoaded = true;
        }
    }

}
