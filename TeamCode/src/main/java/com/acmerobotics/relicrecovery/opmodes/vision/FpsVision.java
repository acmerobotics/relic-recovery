package com.acmerobotics.relicrecovery.opmodes.vision;

import android.graphics.Bitmap;

import com.acmerobotics.library.cameraoverlay.CameraStreamServer;
import com.acmerobotics.relicrecovery.vision.FpsTracker;
import com.acmerobotics.relicrecovery.vision.Overlay;
import com.acmerobotics.relicrecovery.vision.Tracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.acmerobotics.relicrecovery.vision.VuforiaCamera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

/**
 * Created by ryanbrott on 9/23/17.
 */
@TeleOp(name = "FPS Vision", group = "vision")
public class FpsVision extends OpMode {
    private VuforiaCamera camera;
    private FpsTracker fpsTracker;
    private CameraStreamServer streamServer;
    private Bitmap bitmap;
    private Mat rgba;

    @Override
    public void init() {
        streamServer = new CameraStreamServer();
        camera = new VuforiaCamera();
        fpsTracker = new FpsTracker();
        camera.addTracker(fpsTracker);
        camera.addTracker(new Tracker() {
            @Override
            public void init(VisionCamera camera) {

            }

            @Override
            public void processFrame(Mat frame, double timestamp) {
                if (bitmap == null) {
                    bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
                    rgba = new Mat();
                }
                Imgproc.cvtColor(frame, rgba, Imgproc.COLOR_BGR2RGBA);
                Utils.matToBitmap(rgba, bitmap);
                streamServer.send(bitmap);
            }

            @Override
            public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {

            }
        });
        camera.initialize();
    }

    @Override
    public void loop() {

    }
}
