package com.acmerobotics.cameraoverlay.message

class PingCommand: Command<Unit>(MessageType.PING) {
    override fun payloadToByteArray(): ByteArray = ByteArray(0)

    override fun payloadFromByteArray(byteArray: ByteArray) {}

    override fun getResponseType(): Class<out Response<Unit>> {
        return PongResponse::class.java
    }

}