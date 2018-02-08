package com.acmerobotics.library.cameraoverlay;

import android.graphics.Bitmap;
import android.util.Log;

import com.acmerobotics.library.cameraoverlay.message.Message;
import com.acmerobotics.library.cameraoverlay.message.PingCommand;
import com.acmerobotics.library.cameraoverlay.message.PongResponse;
import com.acmerobotics.library.cameraoverlay.message.ReceiveFrameResponse;
import com.acmerobotics.library.cameraoverlay.message.RequestFrameCommand;
import com.acmerobotics.relicrecovery.vision.MatOverlay;
import com.acmerobotics.relicrecovery.vision.Overlay;
import com.acmerobotics.relicrecovery.vision.Tracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;
import org.opencv.android.Utils;
import org.opencv.core.Mat;
import org.opencv.imgproc.Imgproc;

import java.io.IOException;
import java.io.InputStream;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class CameraStreamServer implements Runnable, OpModeManagerNotifier.Notifications {
    public static final String TAG = "CameraStreamServer";
    public static final int PORT = 9000;
    public static final int CAPACITY = 1;

    private ServerSocket serverSocket;
    private BlockingQueue<Bitmap> queue;
    private Socket socket;
    private InputStream inputStream;
    private OutputStream outputStream;
    private ExecutorService executorService;
    private OpModeManagerImpl opModeManager;

    private Bitmap bitmap;
    private Mat temp, rgba;
    private VisionCamera camera;
    private Tracker tracker = new Tracker() {
        @Override
        public void init(VisionCamera camera) {
            CameraStreamServer.this.camera = camera;
        }

        @Override
        public void processFrame(Mat frame, double timestamp) {
            if (bitmap == null) {
                bitmap = Bitmap.createBitmap(frame.width(), frame.height(), Bitmap.Config.ARGB_8888);
                temp = new Mat();
                rgba = new Mat();
            }
            frame.copyTo(temp);
            MatOverlay overlay = new MatOverlay(temp);
            for (Tracker tracker : camera.getTrackers()) {
                if (this == tracker) break;
                tracker.drawOverlay(overlay, temp.cols(), temp.rows(), true);
            }
            Imgproc.cvtColor(temp, rgba, Imgproc.COLOR_BGR2RGBA);
            Utils.matToBitmap(rgba, bitmap);
            send(bitmap);
        }

        @Override
        public void drawOverlay(Overlay overlay, int imageWidth, int imageHeight, boolean debug) {

        }
    };

    public CameraStreamServer() {
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
        try {
            serverSocket = new ServerSocket(PORT);
        } catch (IOException e) {
            Log.w(TAG, e);
        }
        queue = new ArrayBlockingQueue<>(CAPACITY);

        executorService = ThreadPool.newSingleThreadExecutor("camera stream server");
        executorService.submit(this);
    }

    public Tracker getTracker() {
        return tracker;
    }

    public void send(Bitmap bitmap) {
        if (queue.size() < CAPACITY) {
            queue.add(bitmap);
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted() && serverSocket != null) {
            if (socket != null) {
                try {
                    Message message = Message.read(inputStream);
                    if (message instanceof PingCommand) {
                        outputStream.write(new PongResponse().toByteArray());
                        outputStream.flush();
                    } else if (message instanceof RequestFrameCommand) {
                        ReceiveFrameResponse response = new ReceiveFrameResponse();
                        response.set(queue.take());
                        outputStream.write(response.toByteArray());
                        outputStream.flush();
                    } else {
                        Log.w(TAG, "ignoring " + message.getClass().getSimpleName());
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (IOException e) {
                    Log.w(TAG, "process command failed -- reconnecting");
                    socket = null;
                }
            } else {
                try {
                    socket = serverSocket.accept();
                    socket.setSoTimeout(1000);
                    inputStream = socket.getInputStream();
                    outputStream = socket.getOutputStream();
                    Log.i(TAG, "connected to " + socket.getRemoteSocketAddress());
                } catch (IOException e) {
                    Log.w(TAG, "accept failed -- " + e.getMessage());
                }
            }
        }
    }

    public void stop() {
        if (executorService != null) {
            executorService.shutdownNow();
            executorService = null;
        }

        if (socket != null) {
            try {
                socket.close();
                socket = null;
            } catch (IOException e) {
                Log.w(TAG, e);
            }
        }

        if (serverSocket != null) {
            try {
                serverSocket.close();
                serverSocket = null;
            } catch (IOException e) {
                Log.w(TAG, e);
            }
        }

        if (tracker != null) {
            tracker.disable();
            tracker = null;
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
        stop();

        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
            opModeManager = null;
        }
    }
}
