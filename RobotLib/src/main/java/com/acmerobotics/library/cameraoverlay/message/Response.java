package com.acmerobotics.library.cameraoverlay.message;

public abstract class Response<T> extends Message {
    public Response(MessageType type) {
        super(type);
    }

    public abstract void set(T value);
    public abstract T get();
}
