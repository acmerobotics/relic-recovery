package com.acmerobotics.library.cameraoverlay.message;

import android.util.Log;

import java.io.IOException;
import java.io.InputStream;
import java.lang.reflect.InvocationTargetException;
import java.nio.ByteBuffer;
import java.util.HashMap;
import java.util.Map;

public abstract class Message {
    public static final Map<MessageType, Class<? extends Message>> MESSAGE_CLASS_MAP = new HashMap();

    static {
        MESSAGE_CLASS_MAP.put(MessageType.PING, PingCommand.class);
        MESSAGE_CLASS_MAP.put(MessageType.PONG, PongResponse.class);
        MESSAGE_CLASS_MAP.put(MessageType.REQUEST_FRAME, RequestFrameCommand.class);
        MESSAGE_CLASS_MAP.put(MessageType.RECEIVE_FRAME, ReceiveFrameResponse.class);
    }

    public static Message read(InputStream inputStream) throws IOException {
        byte[] headerBytes = new byte[8];
        inputStream.read(headerBytes);
        ByteBuffer headerBuffer = ByteBuffer.wrap(headerBytes);
        int msgTypeInt = headerBuffer.getInt();
        int payloadSize = headerBuffer.getInt();
        byte[] payload = new byte[payloadSize];
        int offset = 0;
        while (offset < payloadSize) {
            int bytesRead = inputStream.read(payload, offset, payloadSize - offset);
            offset += bytesRead;
        }
        MessageType messageType = null;
        for (MessageType type : MessageType.values()) {
            if (type.intVal == msgTypeInt) {
                messageType = type;
                break;
            }
        }
        Class<? extends Message> messageClass = MESSAGE_CLASS_MAP.get(messageType);
        Message message = null;
        try {
            message = (Message) messageClass.getConstructors()[0].newInstance();
            message.payloadFromByteArray(payload);
        } catch (InstantiationException | IllegalAccessException | InvocationTargetException e) {
            Log.wtf("Message", e);
        }
        return message;
    }

    private MessageType type;

    public Message(MessageType type) {
        this.type = type;
    }

    public byte[] toByteArray() {
        byte[] payload = payloadToByteArray();
        ByteBuffer buffer = ByteBuffer.allocate(payload.length + 8);
        buffer.putInt(type.intVal);
        buffer.putInt(payload.length);
        buffer.put(payload);
        return buffer.array();
    }

    public abstract byte[] payloadToByteArray();
    public abstract void payloadFromByteArray(byte[] bytes);
}
