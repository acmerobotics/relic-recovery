package com.acmerobotics.relicrecovery.vision;

import android.app.Activity;
import android.util.Log;
import android.view.ViewGroup;
import android.widget.FrameLayout;
import android.widget.LinearLayout;
import android.widget.RelativeLayout;
import android.widget.ToggleButton;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
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

public class VisionCamera implements OpModeManagerNotifier.Notifications {
    public static final String TAG = "VisionCamera";

    public static final String DEBUG_TOGGLE_TEXT = "DEBUG";

    private VuforiaLocalizer vuforia;
    private VuforiaLocalizer.Parameters vuforiaParams;
    private FrameLayout cameraLayout;
    private RelativeLayout mainLayout;
    private ToggleButton debugToggle;
    private OverlayView overlayView;
    private FrameConsumer frameConsumer;
    private List<Tracker> trackers;
    private File imageDir;
    private int imageNum;
    private boolean initialized;

    private AppUtil appUtil = AppUtil.getInstance();
    private Activity activity;
    private OpModeManagerImpl opModeManager;

    public class FrameConsumer extends Thread {
        public static final String TAG = "FrameConsumer";

        private BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
        private Mat frame;
        private byte[] frameBuffer;
        private boolean running;

        public FrameConsumer(BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue) {
            this.frameQueue = frameQueue;
            this.running = true;
        }

        /** Rescale Vuforia timestamp to the one described in {@link com.acmerobotics.relicrecovery.drive.TimestampedData} */
        private double rescaleVuforiaTimestamp(double timestamp) {
            // TODO: figure this out!
            return timestamp;
        }

        @Override
        public void run() {
            while (running) {
                // grab frames and process them
                if (!frameQueue.isEmpty()) {
                    VuforiaLocalizer.CloseableFrame vuforiaFrame = null;
                    try {
                        vuforiaFrame = frameQueue.take();
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }

                    if (vuforiaFrame == null) {
                        continue;
                    }

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

                            onFrame(this.frame, rescaleVuforiaTimestamp(vuforiaFrame.getTimeStamp()));

                            if (imageDir != null) {
                                String filename = imageNum + ".jpg";
                                Imgcodecs.imwrite(new File(imageDir, filename).getPath(), this.frame);
                                imageNum++;
                            }
                        }
                    }
                    vuforiaFrame.close();
                    Log.i(TAG, "Processed image in " + (System.currentTimeMillis() - startTime) + "ms");
                } else {
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }

        public void terminate() {
            this.running = false;
        }
    }

    public VisionCamera() {
        this.activity = appUtil.getActivity();
        this.trackers = new ArrayList<>();
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(activity);
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
    }

    public synchronized void addTracker(Tracker tracker) {
        this.trackers.add(tracker);

        if (initialized) {
            tracker.init(new VuforiaCameraProperties());
        }

        if (overlayView != null) {
            this.overlayView.addTracker(tracker);
        }
    }

    public void initialize(VuforiaLocalizer.Parameters vuforiaParams) {
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

            appUtil.runOnUiThread(new Runnable() {
                @Override
                public void run() {
                    OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, activity, loaderCallback);
                }
            });

            vuforia = ClassFactory.createVuforiaLocalizer(vuforiaParams);
            this.vuforiaParams = vuforiaParams;

            Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
            vuforia.setFrameQueueCapacity(1);

            if (vuforiaParams.cameraMonitorViewIdParent != 0) {
                this.overlayView = new OverlayView(activity);

                for (Tracker tracker : trackers) {
                    overlayView.addTracker(tracker);
                }

                final Activity activity = appUtil.getActivity();
                activity.runOnUiThread(() -> {
                    LinearLayout cameraMonitorView = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                    cameraLayout = (FrameLayout) cameraMonitorView.getParent();
                    cameraLayout.addView(overlayView);

                    mainLayout = (RelativeLayout) activity.findViewById(R.id.RelativeLayout);
                    debugToggle = new ToggleButton(activity);
                    debugToggle.setText(DEBUG_TOGGLE_TEXT);
                    debugToggle.setTextOff(DEBUG_TOGGLE_TEXT);
                    debugToggle.setTextOn(DEBUG_TOGGLE_TEXT);
                    RelativeLayout.LayoutParams layoutParams = new RelativeLayout.LayoutParams(ViewGroup.LayoutParams.WRAP_CONTENT, ViewGroup.LayoutParams.WRAP_CONTENT);
                    layoutParams.addRule(RelativeLayout.ALIGN_BOTTOM, R.id.textOpMode);
                    layoutParams.addRule(RelativeLayout.ALIGN_PARENT_TOP);
                    layoutParams.addRule(RelativeLayout.ALIGN_PARENT_END);
                    debugToggle.setLayoutParams(layoutParams);
                    debugToggle.setOnCheckedChangeListener((compoundButton, b) -> overlayView.setDebug(b));
                    mainLayout.addView(debugToggle);
                });
            }

            try {
                openCvInitialized.await();
            } catch (InterruptedException e) {
                Log.w(TAG, e);
            }

            for (Tracker tracker : trackers) {
                tracker.init(new VuforiaCameraProperties());
            }

            frameConsumer = new FrameConsumer(vuforia.getFrameQueue());
            frameConsumer.start();

            initialized = true;
        }
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
            appUtil.runOnUiThread(() -> {
                cameraLayout.removeView(overlayView);
                mainLayout.removeView(debugToggle);
                overlayView = null;
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
}
