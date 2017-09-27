package com.acmerobotics.relicrecovery.vision;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.nio.ByteBuffer;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;

/**
 * Created by ryanbrott on 9/23/17.
 */

public class VisionCamera {
    public static final String TAG = "VisionCamera";

    private Context context;
    private VuforiaLocalizer vuforia;
    private FrameLayout parentLayout;
    private OverlayView overlayView;
    private FrameConsumer frameConsumer;
    private List<Tracker> trackers;

    public class FrameConsumer extends Thread {
        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private List<VuforiaLocalizer.CloseableFrame> activeFrames;
        private Mat frame;
        private byte[] frameBuffer;
        private boolean running;

        public FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
            this.running = true;
            this.activeFrames = new ArrayList<>();
        }

        @Override
        public void run() {
            while (running) {
                // grab frames and process them
                if (!frameQueue.isEmpty()) {
                    // TODO handle the frame queue better
                    activeFrames.clear();
                    frameQueue.drainTo(activeFrames);
                    Log.i(TAG, "Drained " + activeFrames.size() + " frames");
                    for (int i = 0; i < activeFrames.size() - 1; i++) {
                        VuforiaLocalizer.CloseableFrame frame = activeFrames.get(i);
                        Log.i(TAG, "Closing " + frame.getTimeStamp());
                        frame.close();
                    }
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = activeFrames.get(activeFrames.size() - 1);
                    Log.i(TAG, "Took " + vuforiaFrame.getTimeStamp());

                    long startTime = System.currentTimeMillis();
                    for (int i = 0; i < vuforiaFrame.getNumImages(); i++) {
                        Image image = vuforiaFrame.getImage(i);
                        if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                            int imageWidth = image.getWidth(), imageHeight = image.getHeight();
                            ByteBuffer byteBuffer = image.getPixels();
                            if (frameBuffer == null) {
                                frameBuffer = new byte[byteBuffer.capacity()];
                            }
                            byteBuffer.get(frameBuffer);
                            if (this.frame == null) {
                                this.frame = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                                overlayView.setImageSize(imageWidth, imageHeight);
                            }
                            this.frame.put(0, 0, frameBuffer);
                            Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                            onFrame(this.frame, vuforiaFrame.getTimeStamp());
                        }
                    }
                    vuforiaFrame.close();
                    Log.i(TAG, "Processed image in " + (System.currentTimeMillis() - startTime) + "ms");
                }
            }
        }

        public void terminate() {
            this.running = false;
        }
    }

    public VisionCamera(Context context, VuforiaLocalizer vuforia) {
        this.context = context;
        this.trackers = new ArrayList<>();
        this.overlayView = new OverlayView(context);
        this.vuforia = vuforia;
    }

    public synchronized void addTracker(Tracker tracker) {
        this.trackers.add(tracker);
        this.overlayView.addTracker(tracker);
    }

    public void initialize() {
        final CountDownLatch openCvInitialized = new CountDownLatch(1);

        final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(context) {
            @Override
            public void onManagerConnected(int status) {
                switch (status) {
                    case LoaderCallbackInterface.SUCCESS:
                    {
                        Log.i(TAG, "OpenCV loaded successfully");
                        openCvInitialized.countDown();
                    } break;
                    default:
                    {
                        super.onManagerConnected(status);
                    } break;
                }
            }
        };

        AppUtil.getInstance().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, context, loaderCallback);
            }
        });

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(VisionConstants.FRAME_QUEUE_CAPACITY);

        frameConsumer = new FrameConsumer(vuforia.getFrameQueue());
        frameConsumer.start();

        final Activity activity = AppUtil.getInstance().getActivity();
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                LinearLayout cameraMonitorView = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                parentLayout = (FrameLayout) cameraMonitorView.getParent();
                parentLayout.addView(overlayView);
            }
        });

        try {
            openCvInitialized.await();
        } catch (InterruptedException e) {
            Log.w(TAG, e);
        }
    }

    private synchronized void onFrame(Mat frame, double timestamp) {
        for (Tracker tracker : trackers) {
            tracker.processFrame(frame, timestamp);
        }

        overlayView.postInvalidate();
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public void close() {
        AppUtil.getInstance().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                parentLayout.removeView(overlayView);
            }
        });

        if (frameConsumer != null) {
            frameConsumer.terminate();
            try {
                frameConsumer.join();
            } catch (InterruptedException e) {
                Log.w(TAG, e);
            }
            frameConsumer = null;
        }
    }
}
