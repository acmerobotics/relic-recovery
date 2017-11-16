package com.acmerobotics.library.dashboard.telemetry;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

/**
 * Created by ryanbrott on 11/12/17.
 */

// TODO: Create an abstract base class for basic telemetry functionality
public class CSVLoggingTelemetry implements Telemetry {
    private Map<String, String> map;
    private boolean autoClear;
    private List<String> header;
    private PrintStream printStream;

    public CSVLoggingTelemetry(File file) {
        map = new LinkedHashMap<>();
        autoClear = true;
        file.mkdirs();
        try {
            printStream = new PrintStream(file);
        } catch (IOException e) {
            e.printStackTrace();
        }
    }

    @Override
    public Item addData(String caption, String format, Object... args) {
        return addData(caption, String.format(Locale.ENGLISH, format, args));
    }

    @Override
    public Item addData(String caption, Object value) {
        map.put(caption, value.toString());
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
        map.clear();
    }

    @Override
    public void clearAll() {
        map.clear();
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
        if (map.size() == 0) return false;
        if (header == null) {
            header = new ArrayList<>();
            StringBuilder headerBuilder = new StringBuilder();
            for (Map.Entry<String, String> entry : map.entrySet()) {
                header.add(entry.getKey());
                headerBuilder.append(entry.getValue());
                headerBuilder.append(",");
            }
            headerBuilder.deleteCharAt(headerBuilder.length() - 1);
            printStream.println(headerBuilder);
        }
        StringBuilder valueBuilder = new StringBuilder();
        for (String key : header) {
            valueBuilder.append(map.get(key));
            valueBuilder.append(",");
        }
        valueBuilder.deleteCharAt(valueBuilder.length() - 1);
        printStream.println(valueBuilder.toString());
        if (autoClear) {
            clear();
        }
        printStream.flush();
        return true;
    }

    @Override
    public Line addLine() {
        throw new UnsupportedOperationException();
    }

    @Override
    public Line addLine(String lineCaption) {
        throw new UnsupportedOperationException();
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

    public void close() {
        if (printStream != null) {
            printStream.close();
            printStream = null;
        }
    }
}
