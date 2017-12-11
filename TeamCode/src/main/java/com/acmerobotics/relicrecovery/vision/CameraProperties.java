package com.acmerobotics.relicrecovery.vision;

/**
 * @author Ryan
 */

public interface CameraProperties {
    /** @return camera's horizontal (along x-axis) focal length in pixels */
    double getHorizontalFocalLengthPx(double imageWidth);
}
