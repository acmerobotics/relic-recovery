package com.acmerobotics.library.cameraoverlay.message;

import android.graphics.Bitmap;

public class RequestFrameCommand extends Command<Bitmap> {
    public RequestFrameCommand() {
        super(MessageType.REQUEST_FRAME);
    }

    @Override
    public Class<? extends Response<Bitmap>> getResponseClass() {
        return ReceiveFrameResponse.class;
    }

    @Override
    public byte[] payloadToByteArray() {
        return new byte[0];
    }

    @Override
    public void payloadFromByteArray(byte[] bytes) {

    }
}
