package com.acmerobotics.relicrecovery.vision;

import android.util.Log;

import com.acmerobotics.library.R;

import org.firstinspires.ftc.robotcore.external.navigation.RelicRecoveryVuMark;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.calib3d.Calib3d;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.DMatch;
import org.opencv.core.KeyPoint;
import org.opencv.core.Mat;
import org.opencv.core.MatOfDMatch;
import org.opencv.core.MatOfKeyPoint;
import org.opencv.core.MatOfPoint2f;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.core.Size;
import org.opencv.features2d.DescriptorExtractor;
import org.opencv.features2d.DescriptorMatcher;
import org.opencv.features2d.FeatureDetector;
import org.opencv.features2d.Features2d;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.util.ArrayList;
import java.util.LinkedList;
import java.util.List;

/**
 * Created by ryanbrott on 12/24/17.
 * https://docs.opencv.org/3.0-beta/doc/py_tutorials/py_feature2d/py_feature_homography/py_feature_homography.html#py-feature-homography
 */

// TODO: clean up a little and potentially add more test points to ensure reliability
public class FeatureDetectionVuMarkTracker extends Tracker {
    public static final int BRIGHTNESS_THRESHOLD = 25;

    public static int RESIZE_WIDTH = 480;

    private FeatureDetector featureDetector;
    private DescriptorExtractor descriptorExtractor;
    private DescriptorMatcher descriptorMatcher;

    private Mat gray, bgr;
    private MatOfKeyPoint objectKeypoints;
    private List<KeyPoint> objectKeypointList;
    private Mat objectDescriptors;
    private MatOfKeyPoint sceneKeypoints;
    private List<KeyPoint> sceneKeypointList;
    private Mat sceneDescriptors;
    private List<MatOfDMatch> latestMatches;
    private List<DMatch> latestGoodMatches;
    private RelicRecoveryVuMark latestVuMark;
    private Point topLeftPoint, bottomLeftPoint;
    private double topLeftValue, bottomLeftValue;

    @Override
    public void init(VisionCamera camera) {
        featureDetector = FeatureDetector.create(FeatureDetector.ORB);
        descriptorExtractor = DescriptorExtractor.create(DescriptorExtractor.ORB);
        descriptorMatcher = DescriptorMatcher.create(DescriptorMatcher.BRUTEFORCE_HAMMING);

        try {
            Mat objectImage = Utils.loadResource(AppUtil.getDefContext(), R.drawable.vumark);
            Log.i("FeatureDetector", "object image: " + objectImage.cols() + "x" + objectImage.rows());
            Imgproc.cvtColor(objectImage, objectImage, Imgproc.COLOR_RGB2GRAY);
            objectKeypoints = new MatOfKeyPoint();
            featureDetector.detect(objectImage, objectKeypoints);
            objectKeypointList = objectKeypoints.toList();
            objectDescriptors = new Mat();
            descriptorExtractor.compute(objectImage, objectKeypoints, objectDescriptors);
            objectImage.release();
        } catch (IOException e) {
            Log.w("FeatureDetector", e);
        }

        gray = new Mat();
        bgr = new Mat();

        sceneKeypoints = new MatOfKeyPoint();
        sceneDescriptors = new Mat();
        latestMatches = new ArrayList<>();
        latestGoodMatches = new ArrayList<>();
    }

    @Override
    public void processFrame(Mat frame, double timestamp) {
        Imgproc.resize(frame, bgr, new Size(RESIZE_WIDTH, ((double) frame.height()) / frame.width() * RESIZE_WIDTH));

        Imgproc.cvtColor(bgr, gray, Imgproc.COLOR_BGR2GRAY);

        featureDetector.detect(gray, sceneKeypoints);
        descriptorExtractor.compute(gray, sceneKeypoints, sceneDescriptors);

        if (sceneDescriptors.empty()) {
            Log.i("FeatureDetector", "empty scene descriptors");
            synchronized (this) {
                latestVuMark = RelicRecoveryVuMark.UNKNOWN;
            }
            return;
        }

        descriptorMatcher.knnMatch(objectDescriptors, sceneDescriptors, latestMatches, 2);

        latestGoodMatches.clear();
        float nndrRatio = 0.7f;

        for (int i = 0; i < latestMatches.size(); i++) {
            MatOfDMatch matOfDMatch = latestMatches.get(i);
            DMatch[] dMatchArray = matOfDMatch.toArray();

            if (dMatchArray.length < 2) {
                continue;
            }

            DMatch m1 = dMatchArray[0];
            DMatch m2 = dMatchArray[1];

            if (m1.distance <= m2.distance * nndrRatio) {
                latestGoodMatches.add(m1);
            }
        }

        Log.i("FeatureDetector", "Found " + latestGoodMatches.size() + " matches");

        Features2d.drawKeypoints(bgr, sceneKeypoints, bgr, new Scalar(0, 255, 255), 0);
        addIntermediate("keypoints", bgr);

        if (latestGoodMatches.isEmpty()) {
            synchronized (this) {
                latestVuMark = RelicRecoveryVuMark.UNKNOWN;
            }
        } else {
            sceneKeypointList = sceneKeypoints.toList();

            LinkedList<Point> objectPoints = new LinkedList<>();
            LinkedList<Point> scenePoints = new LinkedList<>();

            for (int i = 0; i < latestGoodMatches.size(); i++) {
                objectPoints.addLast(objectKeypointList.get(latestGoodMatches.get(i).queryIdx).pt);
                scenePoints.addLast(sceneKeypointList.get(latestGoodMatches.get(i).trainIdx).pt);
            }

            MatOfPoint2f objectMatOfPoint2f = new MatOfPoint2f();
            objectMatOfPoint2f.fromList(objectPoints);
            MatOfPoint2f sceneMatOfPoint2f = new MatOfPoint2f();
            sceneMatOfPoint2f.fromList(scenePoints);

            Mat homography = Calib3d.findHomography(objectMatOfPoint2f, sceneMatOfPoint2f, Calib3d.RANSAC, 3);

            if (homography.empty()) {
                Log.i("FeatureDetector", "no homography");
                synchronized (this) {
                    latestVuMark = RelicRecoveryVuMark.UNKNOWN;
                }
                return;
            }

            // point order: [top left, bottom left]
            // left vumark: [dark, light]
            // center vumark: [light, dark]
            // right vumark: [light, light]
            Mat testPoints = new Mat(2, 2, CvType.CV_32FC2);
            testPoints.put(0, 0, new double[]{133, 178});
            testPoints.put(1, 0, new double[]{133, 214});

            Mat testPointsInScene = new Mat(2, 2, CvType.CV_32FC2);
            Core.perspectiveTransform(testPoints, testPointsInScene, homography);
            topLeftPoint = new Point(testPointsInScene.get(0, 0));
            bottomLeftPoint = new Point(testPointsInScene.get(1, 0));
            Log.i("FeatureDetector", gray.cols() + "x" + gray.rows());
            Log.i("FeatureDetector", "Top left point: " + topLeftPoint);
            Log.i("FeatureDetector", "Bottom left point: " + bottomLeftPoint);
            if (topLeftPoint.x >= 0 && topLeftPoint.y >= 0 && topLeftPoint.x < gray.cols() && topLeftPoint.y < gray.rows()) {
                topLeftValue = gray.get((int) topLeftPoint.y, (int) topLeftPoint.x)[0];
            } else {
                synchronized (this) {
                    latestVuMark = RelicRecoveryVuMark.UNKNOWN;
                }
                return;
            }

            if (bottomLeftPoint.x >= 0 && bottomLeftPoint.y >= 0 && bottomLeftPoint.x < gray.cols() && bottomLeftPoint.y < gray.rows()) {
                bottomLeftValue = gray.get((int) bottomLeftPoint.y, (int) bottomLeftPoint.x)[0];
            } else {
                synchronized (this) {
                    latestVuMark = RelicRecoveryVuMark.UNKNOWN;
                }
                return;
            }

            synchronized (this) {
                if (topLeftValue > BRIGHTNESS_THRESHOLD) {
                    // light; center or right
                    if (bottomLeftValue > BRIGHTNESS_THRESHOLD) {
                        // light; right
                        latestVuMark = RelicRecoveryVuMark.RIGHT;
                    } else {
                        // dark; center
                        latestVuMark = RelicRecoveryVuMark.CENTER;
                    }
                } else {
                    // dark; left
                    latestVuMark = RelicRecoveryVuMark.LEFT;
                }
            }
        }
    }

    @Override
    public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {
        if (latestVuMark != null && latestVuMark != RelicRecoveryVuMark.UNKNOWN) {
            overlay.fillCircle(topLeftPoint, 5, topLeftValue > 127 ? new Scalar(255, 255, 255) : new Scalar(0, 0, 0));
            overlay.fillCircle(bottomLeftPoint, 5, bottomLeftValue > 127 ? new Scalar(255, 255, 255) : new Scalar(0, 0, 0));
//            overlay.putText(String.valueOf(topLeftValue), Overlay.TextAlign.RIGHT, topLeftPoint, new Scalar(0, 0, 255), 50);
//            overlay.putText(String.valueOf(bottomLeftValue), Overlay.TextAlign.RIGHT, bottomLeftPoint, new Scalar(0, 0, 255), 50);
            overlay.putText(latestVuMark.toString(), Overlay.TextAlign.LEFT, new Point(5, 45), new Scalar(0, 0, 255), 45);
        }
    }

    public synchronized RelicRecoveryVuMark getVuMark() {
        return latestVuMark;
    }
}
