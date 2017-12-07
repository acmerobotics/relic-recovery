package com.acmerobotics.relicrecovery.vision;

import com.vuforia.CameraCalibration;
import com.vuforia.CameraDevice;

/**
 * @author Ryan
 */

public class VuforiaCameraProperties implements CameraProperties {
    @Override
    public double getHorizontalFocalLengthPx(double imageWidth) {
        CameraCalibration cameraCalibration = CameraDevice.getInstance().getCameraCalibration();
        double fov = cameraCalibration.getFieldOfViewRads().getData()[0];
        return (imageWidth * 0.5) / Math.tan(0.5 * fov);
    }
}
