package com.acmerobotics.relicrecovery.vision;

import android.view.SurfaceView;
import android.view.ViewGroup;
import android.widget.LinearLayout;

import com.acmerobotics.library.util.TimestampedData;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.opencv.android.CameraBridgeViewBase;
import org.opencv.android.JavaCameraView;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * @author Ryan
 */

public class OpenCVCamera extends VisionCamera implements CameraBridgeViewBase.CvCameraViewListener2 {
    private ViewGroup cameraMonitorView;
    private JavaCameraView cameraView;

    public OpenCVCamera() {
        this(new Parameters());
    }

    public OpenCVCamera(Parameters parameters) {
        super(parameters);
    }

    @Override
    protected void onFrame(Mat frame, double timestamp) {
        super.onFrame(frame, timestamp);

        MatOverlay overlay = new MatOverlay(frame);
        synchronized (trackers) {
            for (Tracker tracker : trackers) {
                overlay.setScalingFactor(1);

                tracker.drawOverlay(overlay, frame.cols(), frame.rows(), true);
            }
        }
    }

    @Override
    protected void doInitialize() {
        cameraView = new JavaCameraView(activity,
                parameters.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT ?
                JavaCameraView.CAMERA_ID_FRONT : JavaCameraView.CAMERA_ID_BACK);
        cameraView.setVisibility(parameters.cameraMonitorViewId == 0 ? SurfaceView.INVISIBLE : SurfaceView.VISIBLE);
        cameraView.setCvCameraViewListener(this);

        appUtil.runOnUiThread(() -> {
            if (parameters.cameraMonitorViewId == 0) {
                cameraMonitorView = (ViewGroup) activity.findViewById(android.R.id.content);
            } else {
                cameraMonitorView = (LinearLayout) activity.findViewById(parameters.cameraMonitorViewId);
            }
            cameraMonitorView.addView(cameraView);
            cameraView.enableView();
        });
    }

    @Override
    public void close() {
        if (cameraView != null) {
            appUtil.runOnUiThread(() -> {
                cameraMonitorView.removeView(cameraView);
                cameraView.disableView();
                cameraView = null;
            });
        }
    }

    @Override
    public Properties getProperties() {
        return new OpenCVProperties();
    }

    @Override
    public void onCameraViewStarted(int width, int height) {

    }

    @Override
    public void onCameraViewStopped() {

    }

    @Override
    public Mat onCameraFrame(CameraBridgeViewBase.CvCameraViewFrame inputFrame) {
        Mat frame = inputFrame.rgba();

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_RGBA2BGR);

        onFrame(frame, TimestampedData.getCurrentTime());

        Imgproc.cvtColor(frame, frame, Imgproc.COLOR_BGR2RGBA);

        return frame;
    }

    private class OpenCVProperties implements Properties {
        @Override
        public double getHorizontalFocalLengthPx(double imageWidth) {
            double fov = Math.toRadians(cameraView.getCamera().getParameters().getHorizontalViewAngle());
            return (imageWidth * 0.5) / Math.tan(0.5 * fov);
        }
    }
}
