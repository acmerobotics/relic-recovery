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
import java.util.Locale;

@Config
public class DynamicJewelTracker extends Tracker {
    public static double DISTANCE_RATIO = 6.0 / 1.875; // distance between centers / radius

    // filter weights
    public static double ECCENTRICITY_WEIGHT = 2;
    public static double SOLIDITY_WEIGHT = 1;
    public static double AREA_WEIGHT = 0;
    public static double DISTANCE_WEIGHT = 0.05;
    public static double AREA_DIFF_WEIGHT = 5;

    private class Jewel {
        public final MatOfPoint contour;
        public final RotatedRect ellipse;
        public final Point centroid;
        public final double eccentricity, solidity, area;

        public Jewel(MatOfPoint contour) {
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

        public double radius() {
            return (ellipse.size.width + ellipse.size.height) / 2;
        }
    }

    private class JewelPair {
        public final Jewel redJewel, blueJewel;
        public final double distanceRatio, areaDiff;

        public JewelPair(Jewel redJewel, Jewel blueJewel) {
            this.redJewel = redJewel;
            this.blueJewel = blueJewel;
            double deltaX = redJewel.centroid.x - blueJewel.centroid.x;
            double deltaY = redJewel.centroid.y - blueJewel.centroid.y;
            double distance = Math.hypot(deltaX, deltaY);
            double avgRadius = (redJewel.radius() + blueJewel.radius()) / 2;
            distanceRatio = distance / avgRadius;
            areaDiff = redJewel.area / (redJewel.area + blueJewel.area);
        }

        /** lower is better */
        public double score() {
            double distanceError = Math.pow(distanceRatio - DISTANCE_RATIO, 2);
            double areaDiffError = Math.pow(areaDiff - 0.5, 2);
            return redJewel.score() + blueJewel.score() + DISTANCE_WEIGHT * distanceError + AREA_DIFF_WEIGHT * areaDiffError;
        }

        public JewelPosition position() {
            return redJewel.centroid.x < blueJewel.centroid.x
                    ? JewelPosition.RED_BLUE : JewelPosition.BLUE_RED;
        }
    }

    public static final Comparator<Jewel> JEWEL_COMPARATOR = (lhs, rhs) -> Double.compare(lhs.score(), rhs.score());
    public static final Comparator<JewelPair> JEWEL_PAIR_COMPARATOR = (lhs, rhs) -> Double.compare(lhs.score(), rhs.score());

    public static int OPEN_KERNEL_SIZE = 5;
    public static int CLOSE_KERNEL_SIZE = 11;

    // red HSV range
    public static int RED_LOWER_HUE = 170, RED_LOWER_SAT = 80, RED_LOWER_VALUE = 120;
    public static int RED_UPPER_HUE = 7, RED_UPPER_SAT = 255, RED_UPPER_VALUE = 255;

    // blue HSV range
    public static int BLUE_LOWER_HUE = 95, BLUE_LOWER_SAT = 80, BLUE_LOWER_VALUE = 120;
    public static int BLUE_UPPER_HUE = 124, BLUE_UPPER_SAT = 255, BLUE_UPPER_VALUE = 255;

    public static int TARGET_AREA = 5200; // px^2
    public static int MIN_AREA = 500;

    public static int RESIZE_WIDTH = 480;

    private Mat resized, hsv, hue, saturation, value, red, blue;
    private Mat temp, redMorphClose, redMorphOpen, blueMorphClose, blueMorphOpen, hierarchy, openKernel, closeKernel;
    private int openKernelSize, closeKernelSize;
    private List<Jewel> lastRedJewels, lastBlueJewels;
    private List<JewelPair> lastJewelPairs;
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

    private List<Jewel> findJewels(AllianceColor color, Mat mask) {
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

        List<Jewel> jewels = new ArrayList<>();
        for (MatOfPoint contour : jewelContours) {
            if (Imgproc.contourArea(contour) >= MIN_AREA && contour.rows() >= 5) {
                jewels.add(new Jewel(contour));
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
            hue = new Mat();
            saturation = new Mat();
            value = new Mat();
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

        Core.extractChannel(hsv, hue, 0);
        Core.extractChannel(hsv, saturation, 1);
        Core.extractChannel(hsv, value, 2);

        addIntermediate("hue", hue);
        addIntermediate("saturation", saturation);
        addIntermediate("value", value);

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

        lastJewelPairs = new ArrayList<>();
        for (Jewel redJewel : lastRedJewels) {
            for (Jewel blueJewel : lastBlueJewels) {
                lastJewelPairs.add(new JewelPair(redJewel, blueJewel));
            }
        }

        Collections.sort(lastJewelPairs, JEWEL_PAIR_COMPARATOR);

        synchronized (this) {
            if (lastJewelPairs.size() > 0) {
                jewelPosition = lastJewelPairs.get(0).position();
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
                Jewel redDetection = lastRedJewels.get(i);
                overlay.strokeContour(redDetection.contour, new Scalar(255, 255, 0), 4);
                overlay.putText(String.valueOf(i), Overlay.TextAlign.CENTER,
                        new Point(redDetection.centroid.x, redDetection.centroid.y), new Scalar(255, 255, 0), 30);
            }

            for (int i = 0; i < lastBlueJewels.size(); i++) {
                Jewel blueDetection = lastBlueJewels.get(i);
                overlay.strokeContour(blueDetection.contour, new Scalar(0, 255, 255), 4);
                overlay.putText(String.valueOf(i), Overlay.TextAlign.CENTER,
                        new Point(blueDetection.centroid.x, blueDetection.centroid.y), new Scalar(0, 255, 255), 30);
            }
        }

        overlay.setScalingFactor(1);

        if (lastJewelPairs != null) {
            for (int i = 0; i < lastRedJewels.size() && i < 4; i++) {
                Jewel redDetection = lastRedJewels.get(i);
                String displayText = String.format(Locale.US, "[%.2f] %.2f,%.2f,%.2fK",
                        redDetection.score(), redDetection.eccentricity, redDetection.solidity, redDetection.area / 1000);
                overlay.putText(displayText, Overlay.TextAlign.LEFT, new Point(5, 5 + 35 * (i + 1)), new Scalar(0, 0, 255), 30);
            }

            for (int i = 0; i < lastBlueJewels.size() && i < 4; i++) {
                Jewel blueDetection = lastBlueJewels.get(i);
                String displayText = String.format(Locale.US, "[%.2f] %.2f,%.2f,%.2fK",
                        blueDetection.score(), blueDetection.eccentricity, blueDetection.solidity, blueDetection.area / 1000);
                overlay.putText(displayText, Overlay.TextAlign.RIGHT, new Point(imageWidth - 5, 5 + 35 * (i + 1)), new Scalar(255, 0, 0), 30);
            }

            for (int i = 0; i < lastJewelPairs.size() && i < 4; i++) {
                JewelPair jewelPair = lastJewelPairs.get(i);
                String displayText = String.format(Locale.US, "[%.2f] %d %d %.2f,%.2f", jewelPair.score(),
                        lastRedJewels.indexOf(jewelPair.redJewel), lastBlueJewels.indexOf(jewelPair.blueJewel), jewelPair.distanceRatio, jewelPair.areaDiff);
                overlay.putText(displayText, Overlay.TextAlign.LEFT, new Point(5, imageHeight - 5 - 35 * i), new Scalar(0, 255, 0), 30);
            }

            if (lastJewelPairs.size() > 0) {
                JewelPair jewelPair = lastJewelPairs.get(0);
                overlay.putText(jewelPair.redJewel.centroid.x + " / " + jewelPair.blueJewel.centroid.x, Overlay.TextAlign.RIGHT, new Point(imageWidth - 5, imageHeight - 75), new Scalar(0, 255, 0), 30);
                overlay.putText(getJewelPosition().toString(), Overlay.TextAlign.RIGHT, new Point(imageWidth - 5, imageHeight - 40), new Scalar(0, 255, 0), 30);
            }
        }
    }

    public synchronized JewelPosition getJewelPosition() {
        return jewelPosition;
    }
}
