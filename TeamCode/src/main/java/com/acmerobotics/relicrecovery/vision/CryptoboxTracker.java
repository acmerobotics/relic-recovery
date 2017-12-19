package com.acmerobotics.relicrecovery.vision;

import android.support.annotation.Nullable;

import com.acmerobotics.library.configuration.AllianceColor;
import com.acmerobotics.library.configuration.Cryptobox;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Angle;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.drive.PoseEstimator;
import com.acmerobotics.relicrecovery.drive.TimestampedData;
import com.acmerobotics.relicrecovery.util.VisionUtil;

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
import java.util.Collections;
import java.util.List;

import static com.acmerobotics.relicrecovery.vision.OldCryptoboxTracker.getMeanRailGap;

/**
 * Created by ryanbrott on 9/23/17.
 */

@Config
public class CryptoboxTracker extends Tracker {
    public interface CryptoboxTrackerListener {
        void onCryptoboxDetection(List<Double> rails, Vector2d estimatedPos, double timestamp);
    }

    public static int HORIZONTAL_OFFSET = -1;
    public static int DISTANCE_OFFSET = 22;

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 0;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 100, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 0;
    public static int BLUE_UPPER_HUE = 124, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    // binary morphology kernel sizes
    public static int OPEN_KERNEL_SIZE = 5;
    public static int HSV_CLOSE_KERNEL_WIDTH = 5;
    public static int HSV_CLOSE_KERNEL_HEIGHT = 41;
    public static int TAPE_CLOSE_KERNEL_WIDTH = 25;
    public static int TAPE_CLOSE_KERNEL_HEIGHT = 9;

    public static double MAX_COS_ANGLE = 0.075;

    public static int TAPE_THRESHOLD = 120;

    public static int RESIZE_WIDTH = 640;

    public static final double ACTUAL_RAIL_GAP = 7.5; // in

    private double actualWidth, actualHeight;
    private Mat resized, hsv, hsvMask, gray, grayMask;
    private Mat hierarchy, openKernel, hsvCloseKernel, tapeCloseKernel;
    private int openKernelSize, hsvCloseKernelWidth, hsvCloseKernelHeight, tapeCloseKernelWidth, tapeCloseKernelHeight;
    private boolean initialized;
    private CameraProperties properties;
    private AllianceColor color;

    private double latestTimestamp;
    private List<Double> latestRails;
    private List<Point> latestAcceptedPoints;
    private List<Point> latestRejectedPoints;

    @Nullable private PoseEstimator poseEstimator;
    private Vector2d latestEstimatedPos;

    private final List<CryptoboxTrackerListener> listeners;

    public CryptoboxTracker(AllianceColor color) {
        this(color, null);
    }

    public CryptoboxTracker(AllianceColor color, PoseEstimator poseEstimator) {
        this.color = color;
        this.poseEstimator = poseEstimator;
        listeners = new ArrayList<>();
    }

    @Override
    public void init(CameraProperties properties) {
        this.properties = properties;
    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        actualWidth = RESIZE_WIDTH;
        actualHeight = (int) (frame.rows() * (actualWidth / frame.cols()));

        if (!initialized) {
            resized = new Mat();
            hsv = new Mat();
            hsvMask = new Mat();
            hierarchy = new Mat();
            gray = new Mat();
            grayMask = new Mat();

            initialized = true;
        }

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

        Imgproc.resize(frame, resized, new Size(actualWidth, actualHeight));

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

        Imgproc.morphologyEx(hsvMask, hsvMask, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(hsvMask, hsvMask, Imgproc.MORPH_CLOSE, hsvCloseKernel);

        Imgproc.cvtColor(resized, gray, Imgproc.COLOR_BGR2GRAY);
        Core.bitwise_and(gray, hsvMask, gray);
        Imgproc.threshold(gray, grayMask, TAPE_THRESHOLD, 255, Imgproc.THRESH_BINARY);

        Imgproc.morphologyEx(grayMask, grayMask, Imgproc.MORPH_OPEN, openKernel);
        Imgproc.morphologyEx(grayMask, grayMask, Imgproc.MORPH_CLOSE, tapeCloseKernel);

        List<MatOfPoint> contours = new ArrayList<>();
        Imgproc.findContours(grayMask, contours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

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
                double vertCosAngle = Vector2d.getCosAngle(v, new Vector2d(1, 0));
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
            rails = VisionUtil.nonMaximumSuppression(rails, 0.5 * getMeanRailGap(rails));
        }

        synchronized (this) {
            latestTimestamp = timestamp;
            latestRails = rails;
            latestAcceptedPoints = points;
            latestRejectedPoints = rejectedPoints;

            updateEstimatedPose();

            synchronized (listeners) {
                for (CryptoboxTrackerListener listener : listeners) {
                    listener.onCryptoboxDetection(latestRails, latestEstimatedPos, latestTimestamp);
                }
            }
        }

    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        if (latestRails != null) {
            overlay.setScalingFactor(imageWidth / actualWidth);

            for (double rail : latestRails) {
                overlay.strokeLine(new Point(rail, 0), new Point(rail, actualHeight), new Scalar(255, 255, 255), 3);
            }

            for (Point point : latestRejectedPoints) {
                overlay.fillCircle(point, 13, new Scalar(0, 0, 0));
                overlay.fillCircle(point, 10, new Scalar(255, 255, 0));
            }

            for (Point point : latestAcceptedPoints) {
                overlay.fillCircle(point, 13, new Scalar(0, 0, 0));
                overlay.fillCircle(point, 10, new Scalar(0, 255, 255));
            }
        }
    }

    public synchronized TimestampedData<List<Double>> getLatestRails() {
        return new TimestampedData<List<Double>>(latestRails, latestTimestamp);
    }

    public synchronized TimestampedData<Vector2d> getLatestPositionEstimate() {
        return new TimestampedData<Vector2d>(latestEstimatedPos, latestTimestamp);
    }

    private Vector2d getPosFromRails(List<Double> rails) {
        double focalLengthPx = properties.getHorizontalFocalLengthPx(actualWidth);

        // TODO: maybe refactor into some functions
        // generally make less awful
        double distance = Double.NaN, offset = Double.NaN;
        if (rails.size() > 1) {
            double meanRailGap = getMeanRailGap(rails);
            distance = (ACTUAL_RAIL_GAP * focalLengthPx) / meanRailGap;
            if (rails.size() == 4) {
                double center = (Collections.max(rails) + Collections.min(rails)) / 2.0;
                offset = ((0.5 * actualWidth - center) * distance) / focalLengthPx;
            }
        }
        return new Vector2d(distance - DISTANCE_OFFSET, offset - HORIZONTAL_OFFSET);
    }

    public static Vector2d getFieldPositionFromCryptoRelativePosition(Cryptobox cryptobox, Vector2d cryptoRelPos) {
        Vector2d cryptoPos = cryptobox.getPose().pos();
        switch (cryptobox) {
            case NEAR_BLUE:
                return new Vector2d(cryptoPos.x() + cryptoRelPos.y(), cryptoPos.y() + cryptoRelPos.x());
            case NEAR_RED:
                return new Vector2d(cryptoPos.x() + cryptoRelPos.y(), cryptoPos.y() - cryptoRelPos.x());
            case FAR_BLUE:
            case FAR_RED:
                // intentional fall-through
                return cryptoPos.added(cryptoRelPos);
        }
        throw new RuntimeException("Invalid cryptobox!");
    }

    private void updateEstimatedPose() {
        if (poseEstimator == null) {
            // simple, heuristic-based pose estimation
            List<Double> rails = new ArrayList<>(latestRails);
            if (rails.size() == 2 || rails.size() == 3) {
                double meanRailGap = getMeanRailGap(rails);
                if (rails.get(0) < meanRailGap) {
                    while (actualWidth - rails.get(rails.size() - 1) > meanRailGap && rails.size() < 4) {
                        // add extra rail on the left
                        rails.add(0, rails.get(0) - meanRailGap);
                    }
                } else if (actualWidth - rails.get(rails.size() - 1) < meanRailGap) {
                    while (rails.get(0) > meanRailGap && rails.size() < 4) {
                        // add extra rail on the right
                        rails.add(rails.get(rails.size() - 1) + meanRailGap);
                    }
                }
            }

            // calculate the distance and horizontal offset of the cryptobox
            // TODO: there are some issues here with coordinate reference frames
            // this version returns a cryptobox-relative coordinate while the other gives a field centric one
            latestEstimatedPos = getPosFromRails(rails);
        } else {
            // advanced pose estimation
            Pose2d robotPose = poseEstimator.getPose();
            Cryptobox cryptobox;
            if (color == AllianceColor.RED) {
                cryptobox = Math.abs(Angle.norm(robotPose.heading() - Math.PI)) < Math.PI / 2 ?
                        Cryptobox.FAR_RED : Cryptobox.NEAR_RED;
            } else {
                cryptobox = Math.abs(Angle.norm(robotPose.heading() - Math.PI)) < Math.PI / 2 ?
                        Cryptobox.FAR_BLUE : Cryptobox.NEAR_BLUE;
            }
            if (latestRails.size() == 4) {
                // we're good
                latestEstimatedPos = getFieldPositionFromCryptoRelativePosition(
                        cryptobox, getPosFromRails(latestRails));
            } else if (latestRails.size() < 2 || latestRails.size() > 4) {
                // uh-oh
                latestEstimatedPos = new Vector2d(Double.NaN, Double.NaN);
            } else {
                // fancy stuff
                double meanRailGap = getMeanRailGap(latestRails);
                int numEstimatedRails = 4 - latestRails.size();
                double bestError = Double.MAX_VALUE;
                Vector2d bestPos = null;
                for (int leftRailsToAdd = 0; leftRailsToAdd <= numEstimatedRails; leftRailsToAdd++) {
                    List<Double> rails = new ArrayList<>(latestRails);
                    int rightRailsToAdd = numEstimatedRails - leftRailsToAdd;
                    for (int i = 0; i < leftRailsToAdd; i++) {
                        // add extra rail on the left
                        rails.add(0, rails.get(0) - meanRailGap);
                    }
                    for (int i = 0; i < rightRailsToAdd; i++) {
                        // add extra rail on the right
                        rails.add(rails.get(rails.size() - 1) + meanRailGap);
                    }
                    Vector2d pos = getFieldPositionFromCryptoRelativePosition(
                            cryptobox, getPosFromRails(rails));
                    double error = robotPose.pos().added(pos.negated()).norm();
                    if (error < bestError) {
                        bestError = error;
                        bestPos = pos;
                    }
                }
                latestEstimatedPos = bestPos;
            }
        }
    }

    public void addListener(CryptoboxTrackerListener listener) {
        synchronized (listeners) {
            listeners.add(listener);
        }
    }

    public void removeListener(CryptoboxTrackerListener listener) {
        synchronized (listeners) {
            listeners.remove(listener);
        }
    }
}
