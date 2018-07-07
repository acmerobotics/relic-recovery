package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.splinelib.Vector2d;

public interface Localizer {
    Vector2d update();
    void setEstimatedPosition(Vector2d position);
}
