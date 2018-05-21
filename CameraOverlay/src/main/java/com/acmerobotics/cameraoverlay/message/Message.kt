package com.acmerobotics.cameraoverlay.message

import java.io.InputStream
import java.nio.ByteBuffer

abstract class Message(private val type: MessageType) {
    companion object {
        private val MESSAGE_CLASS_MAP = mapOf<MessageType, Class<out Message>>(
                MessageType.PING to PingCommand::class.java,
                MessageType.PONG to PongResponse::class.java,
                MessageType.REQUEST_FRAME to RequestFrameCommand::class.java,
                MessageType.RECEIVE_FRAME to ReceiveFrameResponse::class.java
        )

        fun read(inputStream: InputStream): Message {
            val headerBytes = ByteArray(8)
            inputStream.read(headerBytes)
            val headerBuffer = ByteBuffer.wrap(headerBytes)
            val msgTypeInt = headerBuffer.int
            val payloadSize = headerBuffer.int
            val payload = ByteArray(payloadSize)
            var offset = 0;
            while (offset < payloadSize) {
                val bytesRead = inputStream.read(payload, offset, payloadSize - offset)
                offset += bytesRead
            }
            val msgType = MessageType.values().first { it.intVal == msgTypeInt }
            val msgClass = MESSAGE_CLASS_MAP[msgType]
            val message = msgClass?.constructors?.first()?.newInstance() as Message
            message.payloadFromByteArray(payload)
            return message
        }
    }

    fun toByteArray(): ByteArray {
        val payloadByteArray = payloadToByteArray()
        val buffer = ByteBuffer.allocate(payloadByteArray.size + 8)
        buffer.putInt(type.intVal)
        buffer.putInt(payloadByteArray.size)
        buffer.put(payloadByteArray)
        return buffer.array()
    }

    abstract fun payloadToByteArray(): ByteArray
    abstract fun payloadFromByteArray(byteArray: ByteArray)
}