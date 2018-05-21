package com.acmerobotics.cameraoverlay.message

abstract class Command<T>(type: MessageType): Message(type) {
    abstract fun getResponseType(): Class<out Response<T>>
}