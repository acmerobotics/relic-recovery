package com.acmerobotics.relicrecovery.vision;

import android.content.Context;
import android.graphics.Canvas;
import android.view.View;

import java.util.ArrayList;
import java.util.List;

/**
 * Created by ryanbrott on 9/23/17.
 */
public class OverlayView extends View {
    private List<Tracker> trackers;
    private int imageHeight, imageWidth;
    private boolean debug;

    public OverlayView(Context context) {
        super(context);

        this.trackers = new ArrayList<>();
    }

    public void setImageSize(int width, int height) {
        this.imageWidth = width;
        this.imageHeight = height;
    }

    public void addTracker(Tracker tracker) {
        this.trackers.add(tracker);
    }

    public boolean getDebug() {
        return debug;
    }

    public void setDebug(boolean debug) {
        this.debug = debug;
    }

    @Override
    protected void onDraw(Canvas canvas) {
        super.onDraw(canvas);

        // compute the portion of the canvas that overlaps the camera preview
        int canvasWidth = canvas.getWidth(), canvasHeight = canvas.getHeight();
        float canvasAspectRatio = (float) canvasWidth / canvasHeight;

        float imageAspectRatio = (float) imageHeight / imageWidth;

        int overlayWidth, overlayHeight;
        if (canvasAspectRatio > imageAspectRatio) {
            // width is bigger
            overlayWidth = canvasHeight;
            overlayHeight = (int) (canvasHeight * imageAspectRatio);
        } else {
            // height is bigger
            overlayHeight = canvasWidth;
            overlayWidth = (int) (canvasWidth / imageAspectRatio);
        }

        // transform the canvas so that image coordinates can be used
        canvas.translate(canvasWidth / 2.0f, canvasHeight / 2.0f);
        canvas.rotate(90.0f);
        canvas.scale((float) overlayHeight / imageHeight, (float) overlayWidth / imageWidth);
        canvas.translate(-imageWidth / 2.0f, -imageHeight / 2.0f);

        // draw
        for (Tracker tracker : trackers) {
            canvas.save();

            tracker.drawOverlay(new CanvasOverlay(canvas), imageWidth, imageHeight, debug);

            canvas.restore();
        }
    }
}