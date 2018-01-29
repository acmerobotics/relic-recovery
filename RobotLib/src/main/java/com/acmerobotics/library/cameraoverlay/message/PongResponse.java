package com.acmerobotics.library.cameraoverlay.message;

public class PongResponse extends Response {
    public PongResponse() {
        super(MessageType.PONG);
    }

    @Override
    public void set(Object value) {

    }

    @Override
    public Object get() {
        return null;
    }

    @Override
    public byte[] payloadToByteArray() {
        return new byte[0];
    }

    @Override
    public void payloadFromByteArray(byte[] bytes) {

    }
}
