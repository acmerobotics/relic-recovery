package com.acmerobotics.relicrecovery.localization;

import com.acmerobotics.library.localization.Vector2d;

public interface Localizer {
    Vector2d update();
    void setEstimatedPosition(Vector2d position);
}
