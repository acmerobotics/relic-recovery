package com.acmerobotics.library.cameraoverlay.message;

public abstract class Command<T> extends Message {
    public Command(MessageType type) {
        super(type);
    }

    public abstract Class<? extends Response<T>> getResponseClass();
}
