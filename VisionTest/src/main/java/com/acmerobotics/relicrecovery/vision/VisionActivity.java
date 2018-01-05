package com.acmerobotics.relicrecovery.vision;

import android.os.Bundle;
import android.support.v7.app.ActionBar;
import android.support.v7.app.AppCompatActivity;
import android.view.View;

import com.acmerobotics.relicrecovery.configuration.AllianceColor;

import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

public class VisionActivity extends AppCompatActivity implements CameraBridgeViewBase.CvCameraViewListener2 {
    private JavaCameraView cameraView;
    private Tracker tracker;
    private Mat bgr;

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

        // TODO: add support for different detectors
        tracker = new CryptoboxTracker(AllianceColor.BLUE);
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
    }

    @Override
    public void onCameraViewStarted(int width, int height) {
        bgr = new Mat();

        tracker.init(null);
    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat rgba = inputFrame.rgba();

        Imgproc.cvtColor(rgba, bgr, Imgproc.COLOR_RGBA2BGR);

        Overlay overlay = new MatOverlay(rgba);

        tracker.processFrame(bgr, 0);

        tracker.drawOverlay(overlay, bgr.cols(), bgr.rows(), true);

        return rgba;
    }
}
