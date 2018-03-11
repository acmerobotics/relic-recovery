package com.acmerobotics.library.dashboard.telemetry;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpModeManagerNotifier;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.internal.opmode.OpModeManagerImpl;
import org.firstinspires.ftc.robotcore.internal.system.AppUtil;

import java.io.File;
import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.LinkedHashMap;
import java.util.List;
import java.util.Locale;
import java.util.Map;

// TODO: Create an abstract base class for basic telemetry functionality
public class CSVLoggingTelemetry implements Telemetry, OpModeManagerNotifier.Notifications {
    private Map<String, String> map;
    private boolean autoClear;
    private List<String> header;
    private PrintStream printStream;

    private AppUtil appUtil = AppUtil.getInstance();
    private OpModeManagerImpl opModeManager;

    public CSVLoggingTelemetry(File file) {
        map = new LinkedHashMap<>();
        autoClear = true;
        try {
            printStream = new PrintStream(file);
        } catch (IOException e) {
            e.printStackTrace();
        }
        opModeManager = OpModeManagerImpl.getOpModeManagerOfActivity(appUtil.getActivity());
        if (opModeManager != null) {
            opModeManager.registerListener(this);
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
                headerBuilder.append(entry.getKey());
                headerBuilder.append(",");
            }
            headerBuilder.deleteCharAt(headerBuilder.length() - 1);
            if (printStream != null) {
                printStream.println(headerBuilder);
            }
        }
        StringBuilder valueBuilder = new StringBuilder();
        for (String key : header) {
            valueBuilder.append(map.get(key));
            valueBuilder.append(",");
        }
        valueBuilder.deleteCharAt(valueBuilder.length() - 1);
        if (autoClear) {
            clear();
        }
        if (printStream != null) {
            printStream.println(valueBuilder.toString());
        }
        return true;
    }

    @Override
    public Line addLine() {
        throw new UnsupportedOperationException();
    }

    @Override
    public Line addLine(String lineCaption) {
        if (printStream != null) {
            printStream.println(lineCaption);
            printStream.flush();
        }
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

    public void close() {
        if (printStream != null) {
            printStream.flush();
            printStream.close();
            printStream = null;
        }
    }

    @Override
    public void onOpModePreInit(OpMode opMode) {

    }

    @Override
    public void onOpModePreStart(OpMode opMode) {

    }

    @Override
    public void onOpModePostStop(OpMode opMode) {
        close();
        if (opModeManager != null) {
            opModeManager.unregisterListener(this);
        }
    }
}
