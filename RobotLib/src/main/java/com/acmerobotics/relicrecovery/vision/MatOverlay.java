package com.acmerobotics.relicrecovery.vision;

import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;

import java.util.Arrays;
import java.util.List;

public class MatOverlay implements Overlay {
    public static final int FONT = Core.FONT_HERSHEY_DUPLEX;
    public static final int TEXT_THICKNESS = 4;

    private double scalingFactor;
    private Mat dest;

    public MatOverlay(Mat dest) {
        this.scalingFactor = 1;
        this.dest = dest;
    }

    @Override
    public void strokeRect(Rect rect, Scalar color, int thickness) {
        Imgproc.rectangle(dest, new Point(scalingFactor * rect.x, scalingFactor * rect.y),
                new Point(scalingFactor * (rect.x + rect.width), scalingFactor * (rect.y + rect.height)), color, thickness);
    }

    @Override
    public void fillRect(Rect rect, Scalar color) {
        strokeRect(rect, color, Core.FILLED);
    }

    @Override
    public void strokeLine(Point p1, Point p2, Scalar color, int thickness) {
        Imgproc.line(dest, new Point(scalingFactor * p1.x, scalingFactor * p1.y),
                new Point(scalingFactor * p2.x, scalingFactor * p2.y), color, thickness);
    }

    @Override
    public void putText(String text, TextAlign align, Point org, Scalar color, int fontSize) {
        Point newOrg = new Point(scalingFactor * org.x, scalingFactor * org.y);
        Size textSize = Imgproc.getTextSize(text, FONT, 1, TEXT_THICKNESS, new int[]{(int) newOrg.y});
        double fontScale = fontSize / textSize.height;
        double textWidth = fontScale * textSize.width;
        switch (align) {
            case LEFT:
                break;
            case CENTER:
                newOrg.x -= textWidth / 2;
                break;
            case RIGHT:
                newOrg.x -= textWidth;
                break;
        }
        Imgproc.putText(dest, text, newOrg, FONT, fontScale, color, TEXT_THICKNESS);
    }

    @Override
    public void strokeContour(MatOfPoint contour, Scalar color, int thickness) {
        List<Point> contourPoints = contour.toList();
        for (Point point : contourPoints) {
            point.x *= scalingFactor;
            point.y *= scalingFactor;
        }
        MatOfPoint scaledContour = new MatOfPoint();
        scaledContour.fromList(contourPoints);
        Imgproc.drawContours(dest, Arrays.asList(scaledContour), 0, color, thickness);
    }

    @Override
    public void fillContour(MatOfPoint contour, Scalar color) {
        strokeContour(contour, color, Core.FILLED);
    }

    @Override
    public void strokeCircle(Point center, double radius, Scalar color, int thickness) {
        Imgproc.circle(dest, new Point(scalingFactor * center.x, scalingFactor * center.y), (int) (radius * scalingFactor), color, thickness);
    }

    @Override
    public void fillCircle(Point center, double radius, Scalar color) {
        strokeCircle(center, radius, color, Core.FILLED);
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
