package com.acmerobotics.library.cameraoverlay.message;

public enum MessageType {
    PING(0),
    PONG(1),
    REQUEST_FRAME(2),
    RECEIVE_FRAME(3);

    public final int intVal;

    MessageType(int intVal) {
        this.intVal = intVal;
    }
}
