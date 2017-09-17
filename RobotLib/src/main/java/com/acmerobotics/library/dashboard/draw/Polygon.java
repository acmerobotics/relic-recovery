package com.acmerobotics.library.dashboard.draw;

/**
 * Created by ryanbrott on 8/4/17.
 */

public class Polygon extends CanvasOp {
    private double[] xPoints, yPoints;
    private boolean stroke;

    public Polygon(double[] xPoints, double[] yPoints, boolean stroke) {
        super(Type.POLYGON);

        this.xPoints = xPoints;
        this.yPoints = yPoints;
        this.stroke = stroke;
    }
}
