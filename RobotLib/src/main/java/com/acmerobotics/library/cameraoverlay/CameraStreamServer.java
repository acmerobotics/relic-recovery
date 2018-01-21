package com.acmerobotics.library.cameraoverlay;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.io.OutputStream;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class CameraStreamServer implements Runnable, OpModeManagerNotifier.Notifications {
    public static final int PORT = 9000;
    public static final int BLOCK_SIZE = 1024;
    public static final int CAPACITY = 2;
    public static final int QUALITY = 50;

    private ServerSocket serverSocket;
    private BlockingQueue<Bitmap> queue;
    private Socket socket;
    private OutputStream outputStream;
    private ExecutorService executorService;
    private OpModeManagerImpl opModeManager;
    private volatile boolean started;

    public CameraStreamServer() {
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(AppUtil.getInstance().getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
        }
        try {
            serverSocket = new ServerSocket(PORT);
        } catch (IOException e) {
            Log.w("CameraStreamServer", e);
        }
        queue = new ArrayBlockingQueue<>(CAPACITY);

        executorService = ThreadPool.newSingleThreadExecutor("camera stream server");
        executorService.submit(this);
    }

    public void start() {
        started = true;
    }

    public void send(Bitmap bitmap) {
        if (queue.size() < CAPACITY) {
            queue.add(bitmap);
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            if (socket != null) {
                try {
                    if (socket.isClosed()) {
                        Log.i("CameraStreamServer", "disconnect, attempting to reconnect");
                        serverSocket = null;
                        continue;
                    }

                    if (!started) {
                        Log.i("CameraStreamServer", "waiting for start");
                        Thread.sleep(1000);
                        continue;
                    }

                    // compress bitmap to JPEG
                    Bitmap bitmap = queue.take();
                    ByteArrayOutputStream baos = new ByteArrayOutputStream();
                    bitmap.compress(Bitmap.CompressFormat.JPEG, QUALITY, baos);

                    Log.i("CameraStreamServer", "compressed bitmap");

                    // create message bytes
                    byte[] imageBytes = baos.toByteArray();
                    ByteBuffer header = ByteBuffer.allocate(4);
                    header.order(ByteOrder.LITTLE_ENDIAN);
                    header.putInt(imageBytes.length);

                    Log.i("CameraStreamServer", "preparing to send " + imageBytes.length + " bytes");

                    Log.i("CameraStreamServer", socket.isOutputShutdown() ? "output shutdown" : "output ready");

                    // send message
                    outputStream.write(header.array());

                    Log.i("CameraStreamServer", "wrote header");

                    int offset = 0;
                    while (offset < imageBytes.length) {
                        int bytesToSend = Math.min(BLOCK_SIZE, imageBytes.length - offset);
                        outputStream.write(imageBytes, offset, bytesToSend);
                        outputStream.flush();

                        Log.i("CameraStreamServer", "wrote " + bytesToSend + " bytes");

                        offset += BLOCK_SIZE;
                    }
                    outputStream.flush();
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (IOException e) {
                    Log.w("CameraStreamServer", e);
                }
            } else {
                try {
                    socket = serverSocket.accept();
                    outputStream = socket.getOutputStream();
                    Log.i("CameraStreamServer", "got connection from " + socket.getRemoteSocketAddress());
                } catch (IOException e) {
                    Log.w("CameraStreamServer", e);
                }
            }
        }
    }

    public void stop() {
        if (executorService != null) {
            executorService.shutdownNow();
            executorService = null;
        }

        if (serverSocket != null) {
            try {
                serverSocket.close();
                serverSocket = null;
            } catch (IOException e) {
                Log.w("CameraStreamServer", e);
            }
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
