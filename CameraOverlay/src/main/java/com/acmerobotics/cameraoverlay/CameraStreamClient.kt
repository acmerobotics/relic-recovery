package com.acmerobotics.cameraoverlay

import android.os.Handler
import android.util.Log
import com.acmerobotics.cameraoverlay.message.*
import java.io.IOException
import java.net.Socket
import java.util.concurrent.Executors

class CameraStreamClient(private val handler: Handler): Runnable {
    interface Listener {
        fun onConnect()
        fun onDisconnect()
    }

    companion object {
        const val TAG = "CameraStreamClient"
        const val SERVER_PORT = 9000
        const val SERVER_ADDRESS = "192.168.49.1"
    }

    private var socket = Socket()
    private val mainExecutor = Executors.newSingleThreadExecutor()
    private val heartbeatExecutor = Executors.newSingleThreadExecutor()
    private var listener: Listener? = null

    private val heartbeatRunnable = Runnable {
        while (!Thread.currentThread().isInterrupted) {
            try {
                if (sendAndReceive(PingCommand()) == null) {
                    Log.i(TAG, "failed to send ping -- reconnecting")
                    connect()
                }
                try {
                    Thread.sleep(1000)
                } catch (e: InterruptedException) {
                    Thread.currentThread().interrupt()
                }
            } catch (t: Throwable) {
                Log.wtf(TAG, t)
            }
        }
    }

    fun start() {
        Log.i(TAG, "starting")

        mainExecutor.submit(this)
        heartbeatExecutor.submit(heartbeatRunnable)
    }

    fun setListener(listener: Listener) {
        this.listener = listener
    }

    override fun run() {
        while (!Thread.currentThread().isInterrupted) {
            try {
                readAndDisplayImage()
            } catch (t: Throwable) {
                Log.wtf(TAG, t)
            }
        }
    }

    @Suppress("UNCHECKED_CAST")
    @Synchronized
    private fun <T> sendAndReceive(command: Command<T>): T? {
        return try {
            socket.getOutputStream().write(command.toByteArray())
            socket.getOutputStream().flush()
            val message = Message.read(socket.getInputStream())
            if (command.getResponseType().isInstance(message)) {
                val response = message as Response<T>
                response.get()
            } else {
                Log.w(TAG, "mismatched response -- got ${message.javaClass.simpleName}, expected ${command.getResponseType().simpleName}")
                null
            }
        } catch (e: IOException) {
            Log.w(TAG, "send/receive failed -- " + e.message)
            null
        }
    }


    private fun readAndDisplayImage() {
        val bitmap = sendAndReceive(RequestFrameCommand())
        if (bitmap == null) {
            Log.i(TAG, "failed to read image -- ignoring")
            try {
                Thread.sleep(1000)
            } catch (e: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        } else {
            val message = handler.obtainMessage(0, bitmap)
            message.sendToTarget()
        }
    }

    @Synchronized
    private fun connect() {
        if (listener != null) {
            listener?.onDisconnect()
        }
        try {
            socket = Socket(SERVER_ADDRESS, SERVER_PORT)
            socket.soTimeout = 1000
            if (socket.isConnected) {
                Log.i(TAG, "connected to ${socket.remoteSocketAddress}")
                listener?.onConnect()
            }
        } catch (e: IOException) {
            // do nothing
            Log.i(TAG, "connect failed -- " + e.message)
            try {
                Thread.sleep(1500)
            } catch (ie: InterruptedException) {
                Thread.currentThread().interrupt()
            }
        }
    }

    fun stop() {
        Log.i(TAG, "stopping")

        if (!mainExecutor.isShutdown) {
            mainExecutor.shutdownNow();
        }

        if (!heartbeatExecutor.isShutdown) {
            heartbeatExecutor.shutdownNow()
        }

        if (!socket.isClosed) {
            socket.close()
        }
    }
}
