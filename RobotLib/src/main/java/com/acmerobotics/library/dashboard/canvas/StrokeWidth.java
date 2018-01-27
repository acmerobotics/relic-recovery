package com.acmerobotics.library.dashboard.canvas;

public class StrokeWidth extends CanvasOp {
    private int width;

    public StrokeWidth(int width) {
        super(Type.STROKE_WIDTH);

        this.width = width;
    }
}
