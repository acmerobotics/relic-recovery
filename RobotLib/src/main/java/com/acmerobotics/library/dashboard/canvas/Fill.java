package com.acmerobotics.library.dashboard.canvas;

public class Fill extends CanvasOp {
    private String color;

    public Fill(String color) {
        super(Type.FILL);

        this.color = color;
    }
}
