package com.acmerobotics.relicrecovery.vision;

import android.app.Activity;
import android.content.Context;
import android.util.Log;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.imgcodecs.Imgcodecs;
import org.opencv.imgproc.Imgproc;

import java.io.File;
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
    private VuforiaLocalizer.Parameters vuforiaParams;
    private FrameLayout parentLayout;
    private OverlayView overlayView;
    private FrameConsumer frameConsumer;
    private List<Tracker> trackers;
    private File imageDir;
    private int imageNum;

    public class FrameConsumer extends Thread {
        public static final String TAG = "FrameConsumer";

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

                                if (overlayView != null) {
                                    overlayView.setImageSize(imageWidth, imageHeight);
                                }
                            }
                            this.frame.put(0, 0, frameBuffer);

                            Imgproc.cvtColor(this.frame, this.frame, Imgproc.COLOR_RGB2BGR);

                            if (vuforiaParams.cameraDirection == VuforiaLocalizer.CameraDirection.FRONT) {
                                Core.flip(this.frame, this.frame, 1);
                            }

                            onFrame(this.frame, vuforiaFrame.getTimeStamp());

                            if (imageDir != null) {
                                String filename = imageNum + ".jpg";
                                Imgcodecs.imwrite(new File(imageDir, filename).getPath(), this.frame);
                                imageNum++;
                            }
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

    public VisionCamera(Context context) {
        this.context = context;
        this.trackers = new ArrayList<>();
    }

    public synchronized void addTracker(Tracker tracker) {
        this.trackers.add(tracker);

        if (overlayView != null) {
            this.overlayView.addTracker(tracker);
        }
    }

    public void initialize(VuforiaLocalizer.Parameters vuforiaParams) {
        final CountDownLatch openCvInitialized = new CountDownLatch(1);

        final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(context) {
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

        AppUtil.getInstance().runOnUiThread(new Runnable() {
            @Override
            public void run() {
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, context, loaderCallback);
            }
        });

        vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams);
        this.vuforiaParams = vuforiaParams;

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(VisionConstants.FRAME_QUEUE_CAPACITY);

        if (vuforiaParams.cameraMonitorViewIdParent != 0) {
            this.overlayView = new OverlayView(context);

            for (Tracker tracker : trackers) {
                overlayView.addTracker(tracker);
            }

            final Activity activity = AppUtil.getInstance().getActivity();
            activity.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    LinearLayout cameraMonitorView = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                    parentLayout = (FrameLayout) cameraMonitorView.getParent();
                    parentLayout.addView(overlayView);
                }
            });
        }

        try {
            openCvInitialized.await();
        } catch (InterruptedException e) {
            Log.w(TAG, e);
        }

        frameConsumer = new FrameConsumer(vuforia.getFrameQueue());
        frameConsumer.start();
    }

    public void setImageDir(File imageDir) {
        this.imageDir = imageDir;
    }

    private synchronized void onFrame(Mat frame, double timestamp) {
        for (Tracker tracker : trackers) {
            tracker.processFrame(frame, timestamp);
        }

        if (overlayView != null) {
            overlayView.postInvalidate();
        }
    }

    public VuforiaLocalizer getVuforia() {
        return this.vuforia;
    }

    public void close() {
        if (overlayView != null) {
            AppUtil.getInstance().runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    parentLayout.removeView(overlayView);
                    overlayView = null;
                }
            });
        }

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
