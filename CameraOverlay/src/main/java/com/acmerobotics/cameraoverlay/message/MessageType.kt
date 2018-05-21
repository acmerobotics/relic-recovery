package com.acmerobotics.cameraoverlay.message

enum class MessageType(val intVal: Int) {
    PING(0),
    PONG(1),
    REQUEST_FRAME(2),
    RECEIVE_FRAME(3)
}