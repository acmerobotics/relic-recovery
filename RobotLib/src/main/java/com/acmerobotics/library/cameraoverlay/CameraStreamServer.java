package com.acmerobotics.library.cameraoverlay;

import android.graphics.Bitmap;
import android.util.Log;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.BufferedOutputStream;
import java.io.ByteArrayOutputStream;
import java.io.IOException;
import java.net.ServerSocket;
import java.net.Socket;
import java.nio.ByteBuffer;
import java.nio.ByteOrder;
import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ArrayBlockingQueue;
import java.util.concurrent.BlockingQueue;
import java.util.concurrent.ExecutorService;

public class CameraStreamServer implements Runnable, OpModeManagerNotifier.Notifications {
    public static final int PORT = 9000;
    public static final int BLOCK_SIZE = 1024;

    private ServerSocket serverSocket;
    private BlockingQueue<Bitmap> queue;
    private List<Socket> clients;
    private ExecutorService socketService, bitmapService;
    private OpModeManagerImpl opModeManager;
    private boolean started;

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
        queue = new ArrayBlockingQueue<>(5);
        clients = new ArrayList<>();

        socketService = ThreadPool.newSingleThreadExecutor("socket service");
        socketService.submit(this);

        bitmapService = ThreadPool.newSingleThreadExecutor("bitmap service");
    }

    public void start() {
        bitmapService.submit(new BitmapConsumer());
        started = true;
    }

    public void send(Bitmap bitmap) {
        if (!started) return;
        synchronized (this) {
            if (queue.size() < 5 && clients.size() > 0) {
                queue.add(bitmap);
            }
        }
    }

    @Override
    public void run() {
        while (!Thread.currentThread().isInterrupted()) {
            try {
                Socket socket = serverSocket.accept();
                Log.i("CameraStreamServer", "got connection from " + socket.getRemoteSocketAddress());
                synchronized (this) {
                    clients.add(socket);
                }
            } catch (IOException e) {
                Log.w("CameraStreamServer", e);
            }
        }
    }

    public void stop() {
        if (socketService != null) {
            socketService.shutdownNow();
            socketService = null;
        }

        if (bitmapService != null) {
            bitmapService.shutdownNow();
            bitmapService = null;
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

    private class BitmapConsumer implements Runnable {
        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                try {
                    Bitmap bitmap = queue.take();
                    ByteArrayOutputStream baos = new ByteArrayOutputStream();
                    bitmap.compress(Bitmap.CompressFormat.JPEG, 50, baos);
                    byte[] imageBytes = baos.toByteArray();
                    ByteBuffer header = ByteBuffer.allocate(4);
                    header.order(ByteOrder.LITTLE_ENDIAN);
                    header.putInt(imageBytes.length);
                    Log.i("CameraStreamServer", "preparing to send " + imageBytes.length + " bytes");
                    synchronized (this) {
                        int i = 0;
                        while (i < clients.size()) {
                            Socket client = clients.get(i);
                            if (client.isClosed()) {
                                clients.remove(i);
                            } else {
                                BufferedOutputStream outputStream = new BufferedOutputStream(client.getOutputStream());
                                outputStream.write(header.array());
                                int offset = 0;
                                while (offset < imageBytes.length) {
                                    int bytesToSend = Math.min(BLOCK_SIZE, imageBytes.length - offset);
                                    Log.i("CameraStreamServer", "write " + bytesToSend + " bytes");
                                    outputStream.write(imageBytes, offset, bytesToSend);
                                    offset += BLOCK_SIZE;
                                }
                                outputStream.flush();
                                i++;
                            }
                        }
                    }
                } catch (InterruptedException e) {
                    Thread.currentThread().interrupt();
                } catch (IOException e) {
                    Log.w("CameraStreamServer", e);
                }
            }
        }
    }
}
