package com.acmerobotics.relicrecovery.vision;

import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;

import com.acmerobotics.velocityvortex.sensors.ExponentialSmoother;

import org.opencv.core.Mat;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class FpsTracker implements Tracker {
    private ExponentialSmoother smoother;
    private double lastTimestamp, avgTimeDelta;
    private Paint paint;

    public FpsTracker() {
        smoother = new ExponentialSmoother(0.2);
        paint = new Paint();
        paint.setColor(Color.YELLOW);
        paint.setStyle(Paint.Style.FILL_AND_STROKE);
        paint.setStrokeWidth(5.0f);
        paint.setTextSize(45.0f);
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        if (lastTimestamp != 0) {
            double timeDelta = timestamp - lastTimestamp;
            avgTimeDelta = smoother.update(timeDelta);
        }
        lastTimestamp = timestamp;
    }

    @Override
    public void drawOverlay(Canvas canvas, int imageWidth, int imageHeight) {
        canvas.drawText(String.format("%.2f FPS", 1 / avgTimeDelta), 5.0f, 45.0f, paint);
    }
}
