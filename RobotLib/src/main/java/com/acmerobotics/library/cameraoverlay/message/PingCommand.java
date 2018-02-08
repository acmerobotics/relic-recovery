package com.acmerobotics.library.cameraoverlay.message;

public class PingCommand extends Command {
    public PingCommand() {
        super(MessageType.PING);
    }

    @Override
    public Class<? extends Response> getResponseClass() {
        return PongResponse.class;
    }

    @Override
    public byte[] payloadToByteArray() {
        return new byte[0];
    }

    @Override
    public void payloadFromByteArray(byte[] bytes) {

    }
}
