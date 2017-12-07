package com.acmerobotics.relicrecovery.util;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Scalar;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

/**
 * @author Ryan
 */

public class VisionUtil {
    private static Mat temp;

    /**
     * Thresholds an HSV image. If the lower hue is greater than the upper hue, it thresholds from
     * 0 to the lower hue and from the upper hue to 180.
     * @param src
     * @param lowerHsv
     * @param upperHsv
     * @param dest
     */
    public synchronized static void smartHsvRange(Mat src, Scalar lowerHsv, Scalar upperHsv, Mat dest) {
        if (lowerHsv.val[0] > upperHsv.val[0]) {
            Core.inRange(src, lowerHsv, new Scalar(180, upperHsv.val[1], upperHsv.val[2]), dest);
            if (temp == null) {
                temp = new Mat();
            }
            Core.inRange(src, new Scalar(0, lowerHsv.val[1], lowerHsv.val[2]), upperHsv, temp);
            Core.bitwise_or(dest, temp, dest);
        } else {
            Core.inRange(src, lowerHsv, upperHsv, dest);
        }
    }

    /**
     * Replace groups of values within the threshold with their means (thin out duplicate
     * detections). Note: this requires a sorted array and preserves the sorting.
     * @link https://www.pyimagesearch.com/2014/11/17/non-maximum-suppression-object-detection-python/
     * @param values
     * @param threshold
     */
    public static List<Double> nonMaximumSuppression(List<Double> values, double threshold) {
        Collections.sort(values);
        List<Double> outputValues = new ArrayList<>();
        double count = 1, total = values.get(0), lastValue = values.remove(0);
        for (double value : values) {
            if (value - lastValue > threshold) {
                outputValues.add(total / count);
                total = value;
                lastValue = value;
                count = 1;
            } else {
                total += value;
                count += 1;
            }
        }
        outputValues.add(total / count);
        return outputValues;
    }
}
