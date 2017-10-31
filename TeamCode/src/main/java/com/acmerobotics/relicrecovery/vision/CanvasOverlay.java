package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Paint;
import android.graphics.Path;

import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;

import java.util.HashMap;
import java.util.Map;

/**
 * @author Ryan
 */

public class CanvasOverlay implements Overlay {
    private static final Map<TextAlign, Paint.Align> PAINT_ALIGN_MAP = new HashMap<>();

    static {
        PAINT_ALIGN_MAP.put(TextAlign.LEFT, Paint.Align.LEFT);
        PAINT_ALIGN_MAP.put(TextAlign.CENTER, Paint.Align.CENTER);
        PAINT_ALIGN_MAP.put(TextAlign.RIGHT, Paint.Align.RIGHT);
    }

    private Canvas canvas;
    private Paint paint;
    private double scalingFactor;

    public CanvasOverlay(Canvas canvas) {
        this.canvas = canvas;
        this.paint = new Paint();
        this.scalingFactor = 1;
    }

    private void internalDrawRect(Rect rect) {
        canvas.drawRect(
                (float) scalingFactor * rect.x,
                (float) scalingFactor * rect.y,
                (float) scalingFactor * (rect.x + rect.width),
                (float) scalingFactor * (rect.y + rect.height),
                paint
        );
    }

    @Override
    public void strokeRect(Rect rect, Scalar color, int thickness) {
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(VisionUtil.getColorFromScalarBGR(color));
        paint.setStrokeWidth(thickness);
        internalDrawRect(rect);
    }

    @Override
    public void fillRect(Rect rect, Scalar color) {
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(VisionUtil.getColorFromScalarBGR(color));
        internalDrawRect(rect);
    }

    @Override
    public void strokeLine(Point p1, Point p2, Scalar color, int thickness) {
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(VisionUtil.getColorFromScalarBGR(color));
        paint.setStrokeWidth(thickness);
        canvas.drawLine(
                (float) (scalingFactor * p1.x),
                (float) (scalingFactor * p1.y),
                (float) (scalingFactor * p2.x),
                (float) (scalingFactor * p2.y),
                paint
        );
    }

    @Override
    public void putText(String text, TextAlign align, Point org, Scalar color, int fontSize) {
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(VisionUtil.getColorFromScalarBGR(color));
        paint.setTextSize(fontSize);
        paint.setTextAlign(PAINT_ALIGN_MAP.get(align));
        canvas.drawText(text, (float) (scalingFactor * org.x), (float) (scalingFactor * org.y), paint);
    }

    private void internalDrawContour(MatOfPoint contour) {
        if (contour.empty()) return;
        Point[] points = contour.toArray();
        Path path = new Path();
        path.moveTo((float) (scalingFactor * points[0].x), (float) (scalingFactor * points[1].y));
        for (int i = 1; i < points.length; i++) {
            path.lineTo((float) (scalingFactor * points[i].x), (float) (scalingFactor * points[i].y));
        }
        path.close();
        canvas.drawPath(path, paint);
    }

    @Override
    public void strokeContour(MatOfPoint contour, Scalar color, int thickness) {
        paint.setStyle(Paint.Style.STROKE);
        paint.setColor(VisionUtil.getColorFromScalarBGR(color));
        paint.setStrokeWidth(thickness);
        internalDrawContour(contour);
    }

    @Override
    public void fillContour(MatOfPoint contour, Scalar color) {
        paint.setStyle(Paint.Style.FILL);
        paint.setColor(VisionUtil.getColorFromScalarBGR(color));
        internalDrawContour(contour);
    }

    @Override
    public double getScalingFactor() {
        return scalingFactor;
    }

    @Override
    public void setScalingFactor(double factor) {
        scalingFactor = factor;
    }
}
