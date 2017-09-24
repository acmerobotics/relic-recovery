package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class VisionUtil {
    public static void drawRect(Canvas canvas, Rect rect, Paint paint) {
        canvas.drawRect(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height, paint);
    }

    public static void drawLine(Canvas canvas, Point p1, Point p2, Paint paint) {
        canvas.drawLine((float) p1.x, (float) p1.y, (float) p2.x, (float) p2.y, paint);
    }

    public static void drawContour(Canvas canvas, MatOfPoint contour, Paint paint) {
        if (contour.empty()) return;
        Point[] points = contour.toArray();
        Path path = new Path();
        path.moveTo((float) points[0].x, (float) points[1].y);
        for (int i = 1; i < points.length; i++) {
            path.lineTo((float) points[i].x, (float) points[i].y);
        }
        path.close();
        canvas.drawPath(path, paint);
    }
}
