package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Paint;

import org.opencv.core.Rect;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class VisionUtil {
    public static void drawRect(Canvas canvas, Rect rect, Paint paint) {
        canvas.drawRect(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height, paint);
    }
}
