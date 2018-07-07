package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.library.util.TimestampedData;
import com.acmerobotics.library.vision.Overlay;
import com.acmerobotics.library.vision.Tracker;
import com.acmerobotics.library.vision.VisionCamera;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.splinelib.Vector2d;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.List;

import static com.acmerobotics.relicrecovery.vision.OldCryptoboxTracker.getMeanRailGap;

@Config
public class CryptoboxTracker extends Tracker {
    public interface Listener {
        void onCryptoboxDetection(List<Double> rails, double timestamp);
    }

    public static double ROTATION_ANGLE = Math.PI / 12;

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 0;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 100, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 0;
    public static int BLUE_UPPER_HUE = 124, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    // binary morphology kernel sizes
    public static int OPEN_KERNEL_SIZE = 5;
    public static int HSV_CLOSE_KERNEL_WIDTH = 3;
    public static int HSV_CLOSE_KERNEL_HEIGHT = 31;
    public static int TAPE_CLOSE_KERNEL_WIDTH = 17;
    public static int TAPE_CLOSE_KERNEL_HEIGHT = 7;

    public static double MAX_COS_ANGLE = 0.075;

    public static int TAPE_THRESHOLD = 120;

    public static int RESIZE_WIDTH = 480;

    public static final double ACTUAL_RAIL_GAP = 7.5; // in

    private double resizedWidth, resizedHeight;
    private Mat resized, hsv, hsvMask, hsvMaskOpen, hsvMaskClose, gray, grayMask, grayCombined, grayMaskOpen, grayMaskClose;
    private Mat hierarchy, openKernel, hsvCloseKernel, tapeCloseKernel, transform;
    private int openKernelSize, hsvCloseKernelWidth, hsvCloseKernelHeight, tapeCloseKernelWidth, tapeCloseKernelHeight;
    private AllianceColor color;

    private double latestTimestamp;
    private List<Double> latestRails;
    private List<Point> latestAcceptedPoints;
    private List<Point> latestRejectedPoints;

    private final List<Listener> listeners;

    public CryptoboxTracker(AllianceColor color) {
        this.color = color;
        listeners = new ArrayList<>();
    }

    @Override
    public void init(VisionCamera camera) {
        resized = new Mat();
        hsv = new Mat();
        hsvMask = new Mat();
        hsvMaskClose = new Mat();
        hsvMaskOpen = new Mat();
        hierarchy = new Mat();
        gray = new Mat();
        grayCombined = new Mat();
        grayMask = new Mat();
        grayMaskClose = new Mat();
        grayMaskOpen = new Mat();
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        resizedWidth = RESIZE_WIDTH;
        resizedHeight = (int) (frame.rows() * (resizedWidth / frame.cols()));

        if (openKernel == null || openKernelSize != OPEN_KERNEL_SIZE) {
            if (openKernel != null) {
                openKernel.release();
            }
            openKernel = Mat.ones(OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE, CvType.CV_8U);
            openKernelSize = OPEN_KERNEL_SIZE;
        }
        
        if (hsvCloseKernel == null || hsvCloseKernelWidth != HSV_CLOSE_KERNEL_WIDTH || hsvCloseKernelHeight != HSV_CLOSE_KERNEL_HEIGHT) {
            if (hsvCloseKernel != null) {
                hsvCloseKernel.release();
            }
            hsvCloseKernel = Mat.ones(HSV_CLOSE_KERNEL_HEIGHT, HSV_CLOSE_KERNEL_WIDTH, CvType.CV_8U);
            hsvCloseKernelWidth = HSV_CLOSE_KERNEL_WIDTH;
            hsvCloseKernelHeight = HSV_CLOSE_KERNEL_HEIGHT;
        }

        if (tapeCloseKernel == null || tapeCloseKernelWidth != TAPE_CLOSE_KERNEL_WIDTH || tapeCloseKernelHeight != TAPE_CLOSE_KERNEL_HEIGHT) {
            if (tapeCloseKernel != null) {
                tapeCloseKernel.release();
            }
            tapeCloseKernel = Mat.ones(TAPE_CLOSE_KERNEL_HEIGHT, TAPE_CLOSE_KERNEL_WIDTH, CvType.CV_8U);
            tapeCloseKernelWidth = TAPE_CLOSE_KERNEL_WIDTH;
            tapeCloseKernelHeight = TAPE_CLOSE_KERNEL_HEIGHT;
        }

        Imgproc.resize(frame, resized, new Size(resizedWidth, resizedHeight));

        transform = VisionUtil.getZRotationMatrix(resized, ROTATION_ANGLE);

        Imgproc.warpPerspective(resized, resized, transform, resized.size());

        Imgproc.GaussianBlur(resized, resized, new Size(5, 5), 0);

        addIntermediate("blurred", resized);

        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar lowerHsv, upperHsv;
        if (color == AllianceColor.RED) {
            lowerHsv = new Scalar(RED_LOWER_HUE, RED_LOWER_SAT, RED_LOWER_VALUE);
            upperHsv = new Scalar(RED_UPPER_HUE, RED_UPPER_SAT, RED_UPPER_VALUE);
        } else if (color == AllianceColor.BLUE) {
            lowerHsv = new Scalar(BLUE_LOWER_HUE, BLUE_LOWER_SAT, BLUE_LOWER_VALUE);
            upperHsv = new Scalar(BLUE_UPPER_HUE, BLUE_UPPER_SAT, BLUE_UPPER_VALUE);
        } else {
            throw new RuntimeException("Something went wrong...");
        }

        VisionUtil.smartHsvRange(hsv, lowerHsv, upperHsv, hsvMask);

        addIntermediate("hsv threshold", hsvMask);

        Imgproc.morphologyEx(hsvMask, hsvMaskOpen, Imgproc.MORPH_OPEN, openKernel);

        addIntermediate("hsv start", hsvMaskOpen);

        Imgproc.morphologyEx(hsvMaskOpen, hsvMaskClose, Imgproc.MORPH_CLOSE, hsvCloseKernel);

        addIntermediate("hsv close", hsvMaskClose);

        Imgproc.cvtColor(resized, gray, Imgproc.COLOR_BGR2GRAY);

        addIntermediate("gray", gray);

        Core.bitwise_and(gray, hsvMaskClose, grayCombined);

        addIntermediate("gray combined", grayCombined);

        Imgproc.threshold(grayCombined, grayMask, TAPE_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        addIntermediate("gray threshold", grayMask);

        Imgproc.morphologyEx(grayMask, grayMaskOpen, Imgproc.MORPH_OPEN, openKernel);

        addIntermediate("gray start", grayMaskOpen);

        Imgproc.morphologyEx(grayMaskOpen, grayMaskClose, Imgproc.MORPH_CLOSE, tapeCloseKernel);

        addIntermediate("gray close", grayMaskClose);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(grayMaskClose, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<Point> points = new ArrayList<>();
        for (MatOfPoint contour : contours) {
            Moments m = Imgproc.moments(contour);
            int cx = (int) (m.m10 / m.m00);
            int cy = (int) (m.m01 / m.m00);
            points.add(new Point(cx, cy));
        }

        List<Point> verticalPoints = new ArrayList<>();
        for (int i = 0; i < points.size(); i++) {
            Point pt1 = points.get(i);
            for (int j = i + 1; j < points.size(); j++) {
                Point pt2 = points.get(j);
                Vector2d v = new Vector2d(pt2.x - pt1.x, pt2.y - pt1.y);
                double vertCosAngle = v.x() / v.norm();
                if (Math.abs(vertCosAngle) < MAX_COS_ANGLE) {
                    if (!verticalPoints.contains(pt1)) {
                        verticalPoints.add(pt1);
                    }
                    if (!verticalPoints.contains(pt2)) {
                        verticalPoints.add(pt2);
                    }
                }
            }
        }

        List<Point> rejectedPoints = new ArrayList<>();
        for (Point point : points) {
            if (!verticalPoints.contains(point)) {
                rejectedPoints.add(point);
            }
        }

        List<Double> rails = new ArrayList<>();
        for (Point point : verticalPoints) {
            rails.add(point.x);
        }

        if (rails.size() > 2) {
            // TODO: kind of a hack
            int lastRailSize;
            do {
                lastRailSize = rails.size();
                rails = VisionUtil.nonMaximumSuppression(rails, 0.75 * getMeanRailGap(rails));
            } while (lastRailSize != rails.size() && rails.size() >= 2);
        }

        synchronized (this) {
            latestTimestamp = timestamp;
            latestRails = rails;
            latestAcceptedPoints = verticalPoints;
            latestRejectedPoints = rejectedPoints;

            synchronized (listeners) {
                for (Listener listener : listeners) {
                    listener.onCryptoboxDetection(latestRails, latestTimestamp);
                }
            }
        }
    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        if (latestRails != null) {
            overlay.setScalingFactor(imageWidth / resizedWidth);

            Mat transformInv = transform.inv();

            for (double rail : latestRails) {
                overlay.strokeLine(VisionUtil.transformPoint(new Point(rail, 0), transformInv),
                        VisionUtil.transformPoint(new Point(rail, resizedHeight), transformInv), new Scalar(255, 255, 255), 3);
            }

            for (Point point : VisionUtil.transformPoints(latestRejectedPoints, transformInv)) {
                overlay.fillCircle(point, 13, new Scalar(0, 0, 0));
                overlay.fillCircle(point, 10, new Scalar(255, 255, 0));
            }

            for (Point point : VisionUtil.transformPoints(latestAcceptedPoints, transformInv)) {
                overlay.fillCircle(point, 13, new Scalar(0, 0, 0));
                overlay.fillCircle(point, 10, new Scalar(0, 255, 255));
            }
        }
    }

    public AllianceColor getColor() {
        return color;
    }

    public synchronized TimestampedData<List<Double>> getLatestRails() {
        return new TimestampedData<List<Double>>(latestRails, latestTimestamp);
    }

    public double getResizedWidth() {
        return resizedWidth;
    }

    public double getResizedHeight() {
        return resizedHeight;
    }

    public void addListener(Listener listener) {
        synchronized (listeners) {
            listeners.add(listener);
        }
    }

    public void removeListener(Listener listener) {
        synchronized (listeners) {
            listeners.remove(listener);
        }
    }
}
