/* Copyright (c) 2017 FIRST. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted (subject to the limitations in the disclaimer below) provided that
 * the following conditions are met:
 *
 * Redistributions of source code must retain the above copyright notice, this list
 * of conditions and the following disclaimer.
 *
 * Redistributions in binary form must reproduce the above copyright notice, this
 * list of conditions and the following disclaimer in the documentation and/or
 * other materials provided with the distribution.
 *
 * Neither the name of FIRST nor the names of its contributors may be used to endorse or
 * promote products derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY THIS
 * LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO,
 * THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */
package com.acmerobotics.relicrecovery.opmodes;

import android.app.Activity;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.os.Environment;
import android.util.Log;
import android.view.View;
import android.widget.FrameLayout;
import android.widget.LinearLayout;

import com.acmerobotics.library.dashboard.MultipleTelemetry;
import com.acmerobotics.library.dashboard.RobotDashboard;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.vuforia.Image;
import com.vuforia.PIXEL_FORMAT;
import com.vuforia.Vuforia;

import org.firstinspires.ftc.robotcore.external.ClassFactory;
import org.firstinspires.ftc.robotcore.external.navigation.VuforiaLocalizer;
import org.firstinspires.ftc.robotcore.internal.opengl.AutoConfigGLSurfaceView;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.firstinspires.ftc.teamcode.R;
import org.opencv.android.BaseLoaderCallback;
import org.opencv.android.LoaderCallbackInterface;
import org.opencv.android.OpenCVLoader;
import org.opencv.core.Core;
import org.opencv.core.CvType;
import org.opencv.core.Mat;
import org.opencv.core.Rect;
import org.opencv.core.Scalar;
import org.opencv.imgproc.Imgproc;

import java.io.File;
import java.nio.ByteBuffer;
import java.util.Arrays;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.CountDownLatch;

@TeleOp
public class JewelVision extends LinearOpMode {
    public static final float JEWEL_PLATFORM_ASPECT_RATIO = 2.6f; // width/height
    public static final double COLOR_THRESHOLD = 0.7;

    public static final String TAG = "JewelVision";

    private Mat raw, rgb;
    private byte[] imgData;
    private CountDownLatch openCvInitialized = new CountDownLatch(1);
    private File imageSaveLocation;
    VuforiaLocalizer vuforia;
    BlockingQueue<VuforiaLocalizer.CloseableFrame> frameQueue;
    private int surfaceWidth, surfaceHeight;
    private int imageWidth, imageHeight;
    private Rect jewelPlatformRect, leftJewelRect, rightJewelRect;
    private double leftRed, leftBlue, rightRed, rightBlue;
    private FrameLayout parentLayout;

    private static void drawOpenCvRect(Canvas canvas, Rect rect, Paint paint) {
        canvas.drawRect(rect.x, rect.y, rect.x + rect.width, rect.y + rect.height, paint);
    }

    @Override public void runOpMode() throws InterruptedException {
        telemetry = new MultipleTelemetry(Arrays.asList(telemetry, RobotDashboard.getInstance().getTelemetry()));

        final View overlayView = new View(hardwareMap.appContext) {
            @Override
            protected void onDraw(Canvas canvas) {
                super.onDraw(canvas);

                Paint paint = new Paint();
                paint.setStyle(Paint.Style.STROKE);
                paint.setStrokeWidth(5.0f);

                int canvasWidth = canvas.getWidth(), canvasHeight = canvas.getHeight();
                float canvasWidthHeightRatio = (float) canvasWidth / canvasHeight;

                float surfaceViewWidthHeightRatio = (float) surfaceWidth / surfaceHeight;

                int width, height;
                if (canvasWidthHeightRatio > surfaceViewWidthHeightRatio) {
                    // width is bigger
                    width = canvasHeight;
                    height = (int) (canvasHeight * surfaceViewWidthHeightRatio);
                } else {
                    // height is bigger
                    height = canvasWidth;
                    width = (int) (canvasWidth / surfaceViewWidthHeightRatio);
                }

//                Log.i(TAG, String.format("canvas: %d x %d", canvasWidth, canvasHeight));
//                Log.i(TAG, String.format("view: %d x %d", getWidth(), getHeight()));
//                Log.i(TAG, String.format("image: %d x %d", imageWidth, imageHeight));
//                Log.i(TAG, String.format("surface: %d x %d", surfaceWidth, surfaceHeight));
//                Log.i(TAG, String.format("image_on_canvas: %d x %d", width, height));
//                Log.i(TAG, String.format("scale: %f, %f", (float) height / imageHeight, (float) width / imageWidth));

                canvas.translate(canvasWidth / 2.0f, canvasHeight / 2.0f);
                canvas.rotate(90.0f);
                canvas.scale((float) height / imageHeight, (float) width / imageWidth);
                canvas.translate(-imageWidth / 2.0f, -imageHeight / 2.0f);

                if (jewelPlatformRect != null) {
                    paint.setColor(Color.GREEN);
                    drawOpenCvRect(canvas, jewelPlatformRect, paint);

                    if (leftBlue > COLOR_THRESHOLD) {
                        paint.setColor(Color.BLUE);
                    } else if (leftRed > COLOR_THRESHOLD) {
                        paint.setColor(Color.RED);
                    } else {
                        paint.setColor(Color.GREEN);
                    }
                    drawOpenCvRect(canvas, leftJewelRect, paint);

                    if (rightBlue > COLOR_THRESHOLD) {
                        paint.setColor(Color.BLUE);
                    } else if (rightRed > COLOR_THRESHOLD) {
                        paint.setColor(Color.RED);
                    } else {
                        paint.setColor(Color.GREEN);
                    }
                    drawOpenCvRect(canvas, rightJewelRect, paint);
                }
            }
        };

        imageSaveLocation = new File(Environment.getExternalStorageDirectory(), "ACME");
        imageSaveLocation.mkdirs();

        final BaseLoaderCallback loaderCallback = new BaseLoaderCallback(hardwareMap.appContext) {
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
                OpenCVLoader.initAsync(OpenCVLoader.OPENCV_VERSION_3_3_0, hardwareMap.appContext, loaderCallback);
            }
        });

        this.vuforia = ClassFactory.createVuforiaLocalizer(VisionConstants.VUFORIA_PARAMETERS);

        Vuforia.setFrameFormat(PIXEL_FORMAT.RGB888, true);
        vuforia.setFrameQueueCapacity(10);

        frameQueue = vuforia.getFrameQueue();

        final Activity activity = AppUtil.getInstance().getActivity();
        activity.runOnUiThread(new Runnable() {
            @Override
            public void run() {
                LinearLayout cameraMonitorView = (LinearLayout) activity.findViewById(R.id.cameraMonitorViewId);
                AutoConfigGLSurfaceView surfaceView = (AutoConfigGLSurfaceView) cameraMonitorView.getChildAt(0);
                surfaceWidth = surfaceView.getWidth();
                surfaceHeight = surfaceView.getHeight();
                parentLayout = (FrameLayout) cameraMonitorView.getParent();
                parentLayout.addView(overlayView);
            }
        });

        openCvInitialized.await();

        telemetry.addData(">", "Press Play to start");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Grab frames
            if (!frameQueue.isEmpty()) {
                VuforiaLocalizer.CloseableFrame frame = frameQueue.take();
                for (int i = 0; i < frame.getNumImages(); i++) {
                    Image image = frame.getImage(i);
                    if (image.getFormat() == PIXEL_FORMAT.RGB888) {
                        imageWidth = image.getWidth();
                        imageHeight = image.getHeight();
                        ByteBuffer byteBuffer = image.getPixels();
                        if (imgData == null || imgData.length != byteBuffer.capacity()) {
                            imgData = new byte[byteBuffer.capacity()];
                        }
                        byteBuffer.get(imgData);
                        if (raw == null || raw.width() != imageWidth || raw.height() != imageHeight) {
                            raw = new Mat(imageHeight, imageWidth, CvType.CV_8UC3);
                            rgb = new Mat();

                            int platformWidth = imageWidth / 2;
                            int platformHeight = (int) (platformWidth / JEWEL_PLATFORM_ASPECT_RATIO);
                            int offsetX = (imageWidth - platformWidth) / 2;
                            int offsetY = (imageHeight - platformHeight) / 2;

                            jewelPlatformRect = new Rect(offsetX, offsetY, platformWidth, platformHeight);
                            leftJewelRect = new Rect(offsetX, offsetY, platformHeight, platformHeight);
                            rightJewelRect = new Rect(offsetX + platformWidth - platformHeight, offsetY, platformHeight, platformHeight);
                        }
                        raw.put(0, 0, imgData);
                        Imgproc.cvtColor(raw, rgb, Imgproc.COLOR_RGB2BGR);

//                        Imgcodecs.imwrite(new File(imageSaveLocation, System.currentTimeMillis() + ".jpg").getPath(), rgb);

                        Mat leftJewelCropped = rgb.submat(leftJewelRect);
                        Mat rightJewelCropped = rgb.submat(rightJewelRect);

                        Scalar leftJewelTotalColor = Core.sumElems(leftJewelCropped);
                        Scalar rightJewelTotalColor = Core.sumElems(rightJewelCropped);

                        leftBlue = leftJewelTotalColor.val[0];
                        leftRed = leftJewelTotalColor.val[2];
                        rightBlue = rightJewelTotalColor.val[0];
                        rightRed = rightJewelTotalColor.val[2];

                        double leftTotal = leftBlue + leftRed;
                        double rightTotal = rightBlue + rightRed;

                        if (leftTotal == 0) {
                            leftBlue = 0;
                            leftRed = 0;
                        } else {
                            leftBlue /= leftTotal;
                            leftRed /= leftTotal;
                        }

                        if (rightTotal == 0) {
                            rightBlue = 0;
                            rightRed = 0;
                        } else {
                            rightBlue /= rightTotal;
                            rightRed /= rightTotal;
                        }

                        rgb.release();
                        raw.release();
                    }
                }
                frame.close();
            }

            telemetry.addData("red", "%.2f / %.2f", leftRed, rightRed);
            telemetry.addData("blue", "%.2f / %.2f", leftBlue, rightBlue);

            telemetry.update();

            overlayView.postInvalidate();
        }

        parentLayout.removeView(overlayView);
    }
}
