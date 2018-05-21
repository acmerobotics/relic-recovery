package com.acmerobotics.cameraoverlay.message

abstract class Response<T>(type: MessageType): Message(type) {
    abstract fun set(value: T)
    abstract fun get(): T
}