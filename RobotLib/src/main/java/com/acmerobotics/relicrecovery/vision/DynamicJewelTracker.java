package com.acmerobotics.relicrecovery.vision;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;

import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.MatOfPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Rect;
import org.opencv.core.RotatedRect;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.imgproc.Imgproc;
import org.opencv.imgproc.Moments;

import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

/**
 * Created by ryanbrott on 10/30/17.
 */

@Config
public class DynamicJewelTracker extends Tracker {
    // filter weights
    public static double ECCENTRICITY_WEIGHT = 1;
    public static double SOLIDITY_WEIGHT = 1;
    public static double AREA_WEIGHT = 0;

    private class JewelDetection {
        public final MatOfPoint contour;
        public final RotatedRect ellipse;
        public final Point centroid;
        public final double eccentricity, solidity, area;

        public JewelDetection(MatOfPoint contour) {
            this.contour = contour;

            MatOfPoint2f contour2f = new MatOfPoint2f(contour.toArray());
            this.ellipse = Imgproc.fitEllipse(contour2f);
            this.eccentricity = ellipse.size.width / ellipse.size.height;
            this.area = Imgproc.contourArea(contour);
            Rect rect = Imgproc.boundingRect(contour);
            this.solidity = area / rect.area();

            Moments moments = Imgproc.moments(contour);
            centroid = new Point(moments.get_m10() / moments.get_m00(), moments.get_m01() / moments.get_m00());
        }

        /** lower is better */
        public double score() {
            double eccentricityError = Math.pow(Math.log(eccentricity), 2);
            double solidityError = Math.pow(Math.log(solidity) - Math.log(Math.PI / 4), 2);
            double areaError = Math.pow(TARGET_AREA - area, 2);
            return ECCENTRICITY_WEIGHT * eccentricityError + SOLIDITY_WEIGHT * solidityError + AREA_WEIGHT * areaError;
        }
    }

    public static final Comparator<JewelDetection> JEWEL_COMPARATOR = (lhs, rhs) -> Double.compare(lhs.score(), rhs.score());

    public static int OPEN_KERNEL_SIZE = 5;
    public static int CLOSE_KERNEL_SIZE = 11;

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 80;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 95, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 80;
    public static int BLUE_UPPER_HUE = 124, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    public static int TARGET_AREA = 8000; // px^2
    public static int MIN_AREA = 500;

    public static int RESIZE_WIDTH = 480;

    private Mat resized, hsv, red, blue;
    private Mat temp, redMorphClose, redMorphOpen, blueMorphClose, blueMorphOpen, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private List<JewelDetection> lastRedJewels, lastBlueJewels;
    private volatile JewelPosition jewelPosition = JewelPosition.UNKNOWN;
    private double resizedWidth, resizedHeight;

    private void smartHsvRange(Mat src, Scalar lowerHsv, Scalar upperHsv, Mat dest) {
        if (lowerHsv.val[0] > upperHsv.val[0]) {
            Core.inRange(src, lowerHsv, new Scalar(180, upperHsv.val[1], upperHsv.val[2]), dest);
            if (temp == null) {
                temp = new Mat();
            }
            Core.inRange(src, new Scalar(0, lowerHsv.val[1], lowerHsv.val[2]), upperHsv, temp);
            Core.bitwise_or(dest, temp, dest);
        } else {
            Core.inRange(src, lowerHsv, upperHsv, dest);
        }
    }

    private List<JewelDetection> findJewels(AllianceColor color, Mat mask) {
        if (redMorphClose == null) {
            redMorphClose = new Mat();
            redMorphOpen = new Mat();
            blueMorphClose = new Mat();
            blueMorphOpen = new Mat();
            hierarchy = new Mat();
        }

        Mat morphClose, morphOpen;
        String prefix;
        if (color == AllianceColor.RED) {
            prefix = "red";
            morphClose = redMorphClose;
            morphOpen = redMorphOpen;
        } else {
            prefix = "blue";
            morphClose = blueMorphClose;
            morphOpen = blueMorphOpen;
        }

        Imgproc.morphologyEx(mask, morphOpen, Imgproc.MORPH_OPEN, openKernel);

        addIntermediate(prefix + "MorphOpen", morphOpen);

        Imgproc.morphologyEx(morphOpen, morphClose, Imgproc.MORPH_CLOSE, closeKernel);

        addIntermediate(prefix + "MorphClose", morphClose);

        List<MatOfPoint> jewelContours = new ArrayList<>();
        Imgproc.findContours(morphClose, jewelContours, hierarchy, Imgproc.RETR_TREE, Imgproc.CHAIN_APPROX_SIMPLE);

        List<JewelDetection> jewels = new ArrayList<>();
        for (MatOfPoint contour : jewelContours) {
            if (Imgproc.contourArea(contour) >= MIN_AREA && contour.rows() >= 5) {
                jewels.add(new JewelDetection(contour));
            }
        }

        return jewels;
    }

    @Override
    public void init(VisionCamera camera) {

    }

    @Override
    public synchronized void processFrame(Mat frame, double timestamp) {
        resizedWidth = RESIZE_WIDTH;
        resizedHeight = (int) (frame.rows() * (resizedWidth / frame.cols()));

        if (resized == null) {
            resized = new Mat();
            hsv = new Mat();
            red = new Mat();
            blue = new Mat();
        }

        if (openKernel == null || openKernelSize != OPEN_KERNEL_SIZE) {
            if (openKernel != null) {
                openKernel.release();
            }
            openKernel = Mat.ones(OPEN_KERNEL_SIZE, OPEN_KERNEL_SIZE, CvType.CV_8U);
            openKernelSize = OPEN_KERNEL_SIZE;
        }

        if (closeKernel == null || closeKernelSize != CLOSE_KERNEL_SIZE) {
            if (closeKernel != null) {
                closeKernel.release();
            }
            closeKernel = Mat.ones(CLOSE_KERNEL_SIZE, CLOSE_KERNEL_SIZE, CvType.CV_8U);
            closeKernelSize = CLOSE_KERNEL_SIZE;
        }

        Imgproc.resize(frame, resized, new Size(resizedWidth, resizedHeight));

        Imgproc.GaussianBlur(resized, resized, new Size(5, 5), 0);

        addIntermediate("blurred", resized);

        Imgproc.cvtColor(resized, hsv, Imgproc.COLOR_BGR2HSV);

        Scalar redLowerHsv = new Scalar(RED_LOWER_HUE, RED_LOWER_SAT, RED_LOWER_VALUE);
        Scalar redUpperHsv = new Scalar(RED_UPPER_HUE, RED_UPPER_SAT, RED_UPPER_VALUE);
        smartHsvRange(hsv, redLowerHsv, redUpperHsv, red);

        Scalar blueLowerHsv = new Scalar(BLUE_LOWER_HUE, BLUE_LOWER_SAT, BLUE_LOWER_VALUE);
        Scalar blueUpperHsv = new Scalar(BLUE_UPPER_HUE, BLUE_UPPER_SAT, BLUE_UPPER_VALUE);
        smartHsvRange(hsv, blueLowerHsv, blueUpperHsv, blue);

        addIntermediate("red", red);

        lastRedJewels = findJewels(AllianceColor.RED, red);

        addIntermediate("blue", blue);

        lastBlueJewels = findJewels(AllianceColor.BLUE, blue);

        Collections.sort(lastRedJewels, JEWEL_COMPARATOR);
        Collections.sort(lastBlueJewels, JEWEL_COMPARATOR);

        synchronized (this) {
            if (lastRedJewels.size() > 0 && lastBlueJewels.size() > 0) {
                JewelDetection bestRedJewel = lastRedJewels.get(0);
                JewelDetection bestBlueJewel = lastBlueJewels.get(0);
                jewelPosition = bestRedJewel.centroid.x < bestBlueJewel.centroid.x
                        ? JewelPosition.RED_BLUE : JewelPosition.BLUE_RED;
            } else {
                jewelPosition = JewelPosition.UNKNOWN;
            }
        }
    }

    @Override
    public synchronized void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        overlay.setScalingFactor(imageWidth / resizedWidth);

        if (lastRedJewels != null) {
            for (int i = 0; i < lastRedJewels.size(); i++) {
                JewelDetection redDetection = lastRedJewels.get(i);
                overlay.strokeContour(redDetection.contour, new Scalar(0, 255, 255), 2);
                overlay.putText(String.valueOf(i), Overlay.TextAlign.CENTER,
                        new Point(redDetection.centroid.x, redDetection.centroid.y + 15), new Scalar(0, 255, 255), 30);
            }

            for (int i = 0; i < lastBlueJewels.size(); i++) {
                JewelDetection blueDetection = lastBlueJewels.get(i);
                overlay.strokeContour(blueDetection.contour, new Scalar(255, 255, 0), 2);
                overlay.putText(String.valueOf(i), Overlay.TextAlign.CENTER,
                        new Point(blueDetection.centroid.x, blueDetection.centroid.y + 15), new Scalar(255, 255, 0), 30);
            }
        }

        overlay.setScalingFactor(1);

        if (lastRedJewels != null) {
            for (int i = 0; i < lastRedJewels.size(); i++) {
                JewelDetection redDetection = lastRedJewels.get(i);
                String displayText = String.format("[%.2f] %.2f,%.2f,%dK",
                        redDetection.score(), redDetection.eccentricity, redDetection.solidity, (int) (redDetection.area / 1000));
                overlay.putText(displayText, Overlay.TextAlign.LEFT, new Point(5, 5 + 35 * (i + 1)), new Scalar(0, 0, 255), 30);
            }

            for (int i = 0; i < lastBlueJewels.size(); i++) {
                JewelDetection blueDetection = lastBlueJewels.get(i);
                String displayText = String.format("[%.2f] %.2f,%.2f,%dK",
                        blueDetection.score(), blueDetection.eccentricity, blueDetection.solidity, (int) (blueDetection.area / 1000));
                overlay.putText(displayText, Overlay.TextAlign.RIGHT, new Point(imageWidth - 5, 5 + 35 * (i + 1)), new Scalar(255, 0, 0), 30);
            }
        }
    }

    public synchronized JewelPosition getJewelPosition() {
        return jewelPosition;
    }
}
