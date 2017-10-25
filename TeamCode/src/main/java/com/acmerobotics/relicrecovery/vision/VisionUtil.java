package com.acmerobotics.relicrecovery.vision;

import android.graphics.Color;

import org.opencv.core.Scalar;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class VisionUtil {
    // TODO: find a better way to manage colors
    public static int getColorFromScalarBGR(Scalar scalar) {
        return Color.rgb((int) scalar.val[2], (int) scalar.val[1], (int) scalar.val[0]);
    }

}
