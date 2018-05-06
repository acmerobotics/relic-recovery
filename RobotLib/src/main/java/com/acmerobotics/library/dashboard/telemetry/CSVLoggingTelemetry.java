package com.acmerobotics.library.dashboard.telemetry;

import com.acmerobotics.library.util.CSVWriter;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.util.Locale;

// TODO: Create an abstract base class for basic telemetry functionality
public class CSVLoggingTelemetry implements Telemetry {
    private CSVWriter writer;
    private boolean autoClear;

    public CSVLoggingTelemetry(File file) {
        writer = new CSVWriter(file);
        autoClear = true;
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(Locale.ENGLISH, format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        writer.put(caption, value.toString());
        return null;
    }

    @Override
    public <T> Item addData(String caption, Func<T> valueProducer) {
        throw new UnsupportedOperationException();
    }

    @Override
    public <T> Item addData(String caption, String format, Func<T> valueProducer) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeItem(Item item) {
        throw new UnsupportedOperationException();
    }

    @Override
    public void clear() {
        writer.clear();
    }

    @Override
    public void clearAll() {
        writer.clear();
    }

    @Override
    public Object addAction(Runnable action) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean removeAction(Object token) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean update() {
        writer.write();
        if (autoClear) {
            clear();
        }
        return true;
    }

    @Override
    public Line addLine() {
        throw new UnsupportedOperationException();
    }

    @Override
    public Line addLine(String lineCaption) {
        writer.putLine(lineCaption);
        writer.flush();
        return null;
    }

    @Override
    public boolean removeLine(Line line) {
        throw new UnsupportedOperationException();
    }

    @Override
    public boolean isAutoClear() {
        return autoClear;
    }

    @Override
    public void setAutoClear(boolean autoClear) {
        this.autoClear = autoClear;
    }

    @Override
    public int getMsTransmissionInterval() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setMsTransmissionInterval(int msTransmissionInterval) {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getItemSeparator() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setItemSeparator(String itemSeparator) {
        throw new UnsupportedOperationException();
    }

    @Override
    public String getCaptionValueSeparator() {
        throw new UnsupportedOperationException();
    }

    @Override
    public void setCaptionValueSeparator(String captionValueSeparator) {
        throw new UnsupportedOperationException();
    }

    @Override
    public Log log() {
        return null;
    }
}
