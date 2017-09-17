package com.acmerobotics.library.dashboard.draw;

/**
 * Created by ryanbrott on 8/4/17.
 */

public class Fill extends CanvasOp {
    private String color;

    public Fill(String color) {
        super(Type.FILL);

        this.color = color;
    }
}
