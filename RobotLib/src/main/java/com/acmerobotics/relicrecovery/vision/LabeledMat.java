package com.acmerobotics.relicrecovery.vision;

import org.opencv.core.Mat;

/**
 * @author Ryan
 */

public class LabeledMat {
    public final String name;
    public final Mat mat;

    public LabeledMat(String name, Mat mat) {
        this.name = name;
        this.mat = mat;
    }
}
