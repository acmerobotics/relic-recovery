package com.acmerobotics.relicrecovery.vision;

import android.app.Activity;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Mat;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.CountDownLatch;

/**
 * @author Ryan
 */

public abstract class VisionCamera implements OpModeManagerNotifier.Notifications {
    public static final String TAG = "VisionCamera";

    protected List<Tracker> trackers;

    protected AppUtil appUtil = AppUtil.getInstance();
    protected Activity activity;
    protected OpModeManagerImpl opModeManager;

    private boolean initialized;

    public VisionCamera() {
        this.activity = appUtil.getActivity();
        this.trackers = new ArrayList<>();
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    public void initialize() {
        if (!initialized) {
            final CountDownLatch openCvInitialized = new CountDownLatch(1);

            final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(activity) {
                @Override
                public void onManagerConnected(int status) {
                    switch (status) {
                        case LoaderCallbackInterface.SUCCESS: {
                            Log.i(TAG, "OpenCV loaded successfully");
                            openCvInitialized.countDown();
                            break;
                        }
                        default: {
                            super.onManagerConnected(status);
                            break;
                        }
                    }
                }
            };

            appUtil.runOnUiThread(() -> OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, activity, loaderCallback));

            try {
                openCvInitialized.await();
            } catch (InterruptedException e) {
                Log.w(TAG, e);
            }

            for (Tracker tracker : trackers) {
                tracker.init(new VuforiaCameraProperties());
            }

            doInitialize();

            initialized = true;
        }
    }

    public synchronized void addTracker(Tracker tracker) {
        this.trackers.add(tracker);

        if (initialized) {
            tracker.init(new VuforiaCameraProperties());
        }
    }

    protected void onFrame(Mat frame, double timestamp) {
        for (Tracker tracker : trackers) {
            tracker.internalProcessFrame(frame, timestamp);
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        close();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }

    protected abstract void doInitialize();
    public abstract void close();
}
