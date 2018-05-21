package com.acmerobotics.cameraoverlay.message

class PongResponse: Response<Unit>(MessageType.PONG) {
    override fun set(value: Unit) {}
    override fun get() {}

    override fun payloadToByteArray(): ByteArray = ByteArray(0)

    override fun payloadFromByteArray(byteArray: ByteArray) {}

}