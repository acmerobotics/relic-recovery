package com.acmerobotics.library.dashboard.draw;

/**
 * Created by ryanbrott on 8/4/17.
 */

public class StrokeWidth extends CanvasOp {
    private int width;

    public StrokeWidth(int width) {
        super(Type.STROKE_WIDTH);

        this.width = width;
    }
}
