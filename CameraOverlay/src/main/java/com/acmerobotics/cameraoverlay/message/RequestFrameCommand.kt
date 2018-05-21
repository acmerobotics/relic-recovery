package com.acmerobotics.cameraoverlay.message

import android.graphics.Bitmap

class RequestFrameCommand: Command<Bitmap>(MessageType.REQUEST_FRAME) {
    override fun getResponseType(): Class<out Response<Bitmap>> {
        return ReceiveFrameResponse::class.java
    }

    override fun payloadToByteArray(): ByteArray = ByteArray(0)
    override fun payloadFromByteArray(byteArray: ByteArray) {}

}
