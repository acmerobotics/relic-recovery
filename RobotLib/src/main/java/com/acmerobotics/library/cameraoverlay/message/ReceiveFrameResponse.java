package com.acmerobotics.library.cameraoverlay.message;

import android.graphics.Bitmap;
import android.graphics.BitmapFactory;

import java.io.ByteArrayOutputStream;

public class ReceiveFrameResponse extends Response<Bitmap> {
    private Bitmap bitmap;

    public ReceiveFrameResponse() {
        super(MessageType.RECEIVE_FRAME);
    }

    @Override
    public void set(Bitmap value) {
        bitmap = value;
    }

    @Override
    public Bitmap get() {
        return bitmap;
    }

    @Override
    public byte[] payloadToByteArray() {
        ByteArrayOutputStream outputStream = new ByteArrayOutputStream();
        bitmap.compress(Bitmap.CompressFormat.JPEG, 50, outputStream);
        return outputStream.toByteArray();
    }

    @Override
    public void payloadFromByteArray(byte[] bytes) {
        bitmap = BitmapFactory.decodeByteArray(bytes, 0, bytes.length);
    }
}
