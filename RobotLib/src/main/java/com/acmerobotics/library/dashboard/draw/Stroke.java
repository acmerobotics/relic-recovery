package com.acmerobotics.library.dashboard.draw;

/**
 * Created by ryanbrott on 8/4/17.
 */

public class Stroke extends CanvasOp {
    private String color;

    public Stroke(String color) {
        super(Type.STROKE);

        this.color = color;
    }
}
