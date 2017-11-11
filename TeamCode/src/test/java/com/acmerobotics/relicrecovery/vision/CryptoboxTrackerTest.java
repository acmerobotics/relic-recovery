package com.acmerobotics.relicrecovery.vision;

import org.junit.Ignore;
import org.junit.Test;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;

import java.io.File;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

import static org.junit.Assert.assertEquals;

/**
 * Created by ryanbrott on 10/12/17.
 */

public class CryptoboxTrackerTest {
    @Test
    public void testNonMaximumSuppression() {
        List<Double> values = new ArrayList<>(Arrays.asList(1.0, 1.01, 1.5, 2.0, 3.0, 3.15, 4.0));
        List<Double> actualOutputValues = CryptoboxTracker.nonMaximumSuppression(values, 0.2);
        List<Double> expectedOutputValues = Arrays.asList(1.005, 1.5, 2.0, 3.075, 4.0);

        assertEquals(expectedOutputValues.size(), actualOutputValues.size());
        for (int i = 0; i < expectedOutputValues.size(); i++) {
            assertEquals(expectedOutputValues.get(i), actualOutputValues.get(i), 0.0001);
        }
    }

    @Test
    @Ignore
    public void testSampleImages() {
        System.load("/usr/local/Cellar/opencv/3.3.1/share/OpenCV/java/libopencv_java331.dylib");
//        System.load("R:\\Downloads\\opencv\\build\\java\\x64\\opencv_java331.dll");

        CryptoboxTracker.isUnitTest = true;

        CryptoboxTracker tracker = new CryptoboxTracker(true);

        File imageSourceDir = new File("scripts/cryptobox/images");
//        File imageSourceDir = new File("/Users/ryanbrott/Downloads/Cryptoboxes");
        File imageOutputDir = new File("scripts/cryptobox/output");
        imageOutputDir.mkdirs();

        for (File imageFile : imageSourceDir.listFiles()) {
            if (imageFile.getName().startsWith(".")) {
                continue;
            }
            System.out.println("processing " + imageFile.getName());

            Mat image = Imgcodecs.imread(imageFile.getAbsolutePath());

            tracker.processFrame(image, 0);

            MatOverlay overlay = new MatOverlay(image);
            tracker.drawOverlay(overlay, image.cols(), image.rows());

            Imgcodecs.imwrite(new File(imageOutputDir, imageFile.getName()).getAbsolutePath(), image);
        }
    }
}
