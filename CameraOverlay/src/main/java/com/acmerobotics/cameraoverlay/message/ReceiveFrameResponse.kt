package com.acmerobotics.cameraoverlay.message

import android.graphics.Bitmap
import android.graphics.BitmapFactory
import java.io.ByteArrayOutputStream

class ReceiveFrameResponse : Response<Bitmap>(MessageType.RECEIVE_FRAME) {
    private lateinit var bitmap: Bitmap

    override fun payloadFromByteArray(byteArray: ByteArray) {
        bitmap = BitmapFactory.decodeByteArray(byteArray, 0, byteArray.size)
    }

    override fun payloadToByteArray(): ByteArray {
        val outputStream = ByteArrayOutputStream()
        bitmap.compress(Bitmap.CompressFormat.JPEG, 80, outputStream)
        return outputStream.toByteArray()
    }

    override fun set(value: Bitmap) {
        this.bitmap = value
    }

    override fun get(): Bitmap {
        return bitmap
    }
}