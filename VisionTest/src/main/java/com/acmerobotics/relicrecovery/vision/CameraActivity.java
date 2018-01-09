package com.acmerobotics.relicrecovery.vision;

import android.os.Bundle;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.util.Log;
import android.view.MotionEvent;
import android.view.View;
import android.view.WindowManager;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.Mat;
import org.opencv.core.Point;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.util.List;

public class CameraActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2, View.OnTouchListener {
    private JavaCameraView cameraView;
    private Tracker tracker;
    private int intermediateIndex;
    private Mat bgra, temp;
    private RobotDashboard dashboard;

    private BaseLoaderCallback loaderCallback = new BaseLoaderCallback(this) {
        @Override
        public void onManagerConnected(int status) {
            switch (status) {
                case LoaderCallbackInterface.SUCCESS: {
                    cameraView.enableView();
                    break;
                }
                default: {
                    super.onManagerConnected(status);
                    break;
                }
            }
        }
    };

    @Override
    public void onCreate(Bundle savedInstanceState) {
        super.onCreate(savedInstanceState);

        getWindow().addFlags(WindowManager.LayoutParams.FLAG_KEEP_SCREEN_ON);

        View decorView = getWindow().getDecorView();
        // Hide the status bar.
        int uiOptions = View.SYSTEM_UI_FLAG_FULLSCREEN;
        decorView.setSystemUiVisibility(uiOptions);
        // Remember that you should never show the action bar if the
        // status bar is hidden, so hide that too if necessary.
        ActionBar actionBar = getSupportActionBar();
        if (actionBar != null) {
            actionBar.hide();
        }

        cameraView = new JavaCameraView(this, CameraBridgeViewBase.CAMERA_ID_BACK);
        setContentView(cameraView);
        cameraView.setVisibility(View.VISIBLE);
        cameraView.setCvCameraViewListener(this);
        cameraView.setOnTouchListener(this);

        String detector = getIntent().getStringExtra("detector");
        if (detector.equals("Red Cryptobox")) {
            tracker = new CryptoboxTracker(AllianceColor.RED);
        } else if (detector.equals("Blue Cryptobox")) {
            tracker = new CryptoboxTracker(AllianceColor.BLUE);
        } else if (detector.equals("Jewel")) {
            tracker = new FixedJewelTracker();
        } else if (detector.equals("Pictograph")) {
            tracker = new FeatureDetectionVuMarkTracker();
        } else {
            Log.wtf("CameraActivity", "Unknown detector: " + detector);
        }

        dashboard = RobotDashboard.open(this, null);
    }

    @Override
    public void onPause() {
        super.onPause();
        if (cameraView != null)
            cameraView.disableView();
    }

    @Override
    public void onResume() {
        super.onResume();
        if (!OpenCVLoader.initDebug()) {
            OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, this, loaderCallback);
        } else {
            loaderCallback.onManagerConnected(LoaderCallbackInterface.SUCCESS);
        }
    }

    public void onDestroy() {
        super.onDestroy();
        if (cameraView != null)
            cameraView.disableView();
        if (dashboard != null)
            dashboard.stop();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        bgra = new Mat();
        temp = new Mat();

        tracker.init(null);
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgba = inputFrame.rgba();

        Imgproc.cvtColor(rgba, bgra, Imgproc.COLOR_RGBA2BGR);

        tracker.processFrame(bgra, 0);

        // wrap properly
        List<LabeledMat> intermediates = tracker.getIntermediates();
        while (intermediateIndex < 0) {
            intermediateIndex += intermediates.size() + 1;
        }
        intermediateIndex = intermediateIndex % (intermediates.size() + 1);

        if (intermediateIndex == 0) {
            Overlay overlay = new MatOverlay(bgra);

            tracker.drawOverlay(overlay, bgra.cols(), bgra.rows(), true);

            Imgproc.cvtColor(bgra, rgba, Imgproc.COLOR_BGRA2RGBA);

            return rgba;
        } else {
            LabeledMat intermediate = intermediates.get(intermediateIndex - 1);

            if (intermediate.mat.channels() == 3) {
                // bgra
                Imgproc.cvtColor(intermediate.mat, temp, Imgproc.COLOR_BGR2RGBA);
            } else {
                // gray
                Imgproc.cvtColor(intermediate.mat, temp, Imgproc.COLOR_GRAY2RGBA);
            }

            Imgproc.resize(temp, temp, rgba.size());

            Imgproc.putText(temp, intermediate.name, new Point(5, 80), Core.FONT_HERSHEY_DUPLEX, 3, new Scalar(0, 255, 0), 2);

            return temp;
        }
    }

    @Override
    public boolean onTouch(View v, MotionEvent event) {
        if (event.getX() < v.getWidth() / 2) {
            // left
            intermediateIndex--;
        } else {
            // right
            intermediateIndex++;
        }

        return false;
    }
}
