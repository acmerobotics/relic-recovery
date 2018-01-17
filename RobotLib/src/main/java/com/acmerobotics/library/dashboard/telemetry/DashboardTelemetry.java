package com.acmerobotics.library.dashboard.telemetry;

import android.support.annotation.Nullable;

import com.acmerobotics.library.dashboard.RobotDashboard;
import com.acmerobotics.library.dashboard.message.Message;
import com.acmerobotics.library.dashboard.message.MessageType;
import com.acmerobotics.library.util.TimestampedData;
import com.qualcomm.robotcore.util.ThreadPool;

import org.firstinspires.ftc.robotcore.external.Func;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.ArrayList;
import java.util.List;
import java.util.concurrent.ExecutorService;

/**
 * @author Ryan
 */

public class DashboardTelemetry implements Telemetry {
    private class TelemetryUpdateThread implements Runnable {
        private Message telemetryMessageToSend;
        private double lastMessageTimestamp;

        @Override
        public void run() {
            while (!Thread.currentThread().isInterrupted()) {
                double currentTime = TimestampedData.getCurrentTime();
                double timeSinceLastTransmission = currentTime - lastMessageTimestamp;
                if (telemetryMessageToSend != null && timeSinceLastTransmission >= (msTransmissionInterval / 1000.0)) {
                    synchronized (this) {
                        dashboard.sendAll(telemetryMessageToSend);
                        lastMessageTimestamp = currentTime;
                        telemetryMessageToSend = null;
                    }
                } else {
                    try {
                        Thread.sleep(5);
                    } catch (InterruptedException e) {
                        Thread.currentThread().interrupt();
                    }
                }
            }
        }

        public synchronized void queueMessage(Message message) {
            telemetryMessageToSend = message;
        }
    }

    public class LogImpl implements Log {
        private transient int capacity = 9;
        private DisplayOrder displayOrder;
        private List<String> lines;

        public LogImpl() {
            this.lines = new ArrayList<>();
        }

        @Override
        public synchronized int getCapacity() {
            return this.capacity;
        }

        @Override
        public synchronized void setCapacity(int capacity) {
            this.capacity = capacity;
        }

        @Override
        public synchronized DisplayOrder getDisplayOrder() {
            return this.displayOrder;
        }

        @Override
        public synchronized void setDisplayOrder(DisplayOrder displayOrder) {
            this.displayOrder = displayOrder;
        }

        @Override
        public synchronized void add(String line) {
            this.lines.add(line);
        }

        @Override
        public synchronized void add(String line, Object... objects) {
            this.add(String.format(line, objects));
        }

        @Override
        public synchronized void clear() {
            this.lines.clear();
        }
    }

    public class LineImpl implements Line {
        private List<ItemImpl> items;

        public LineImpl() {
            this.items = new ArrayList<>();
        }

        @Override
        public synchronized Item addData(String caption, String value, Object... objects) {
            ItemImpl item = new ItemImpl(this);
            item.setCaption(caption);
            item.setValue(value, objects);
            this.items.add(item);
            return item;
        }

        @Override
        public synchronized Item addData(String caption, Object value) {
            ItemImpl item = new ItemImpl(this);
            item.setCaption(caption);
            item.setValue(value);
            this.items.add(item);
            return item;
        }

        @Override
        public synchronized <T> Item addData(String caption, Func<T> func) {
            ItemImpl item = new ItemImpl(this);
            item.setCaption(caption);
            item.setValue(func);
            this.items.add(item);
            return item;
        }

        @Override
        public synchronized <T> Item addData(String caption, String value, Func<T> func) {
            ItemImpl item = new ItemImpl(this);
            item.setCaption(caption);
            item.setValue(value, value, func);
            this.items.add(item);
            return item;
        }
    }

    public class ItemImpl implements Item {
        private transient Line parent;
        private transient boolean retained;

        private String caption, value;

        public ItemImpl(Line parent) {
            this.parent = parent;
        }

        @Override
        public synchronized String getCaption() {
            return this.caption;
        }

        @Override
        public synchronized Item setCaption(String caption) {
            this.caption = caption;
            return this;
        }

        @Override
        public synchronized Item setValue(String value, Object... objects) {
            return setValue(String.format(value, objects));
        }

        @Override
        public synchronized Item setValue(Object value) {
            this.value = value.toString();
            return this;
        }

        @Override
        public synchronized <T> Item setValue(final Func<T> func) {
            throw new UnsupportedOperationException();
        }

        @Override
        public synchronized <T> Item setValue(final String caption, final Func<T> func) {
            throw new UnsupportedOperationException();
        }

        @Override
        public synchronized Item setRetained(@Nullable Boolean aBoolean) {
            this.retained = aBoolean.booleanValue();
            return this;
        }

        @Override
        public synchronized boolean isRetained() {
            return this.retained;
        }

        @Override
        public synchronized Item addData(String caption, String value, Object... objects) {
            this.parent.addData(caption, value, objects);
            return this;
        }

        @Override
        public synchronized Item addData(String caption, Object value) {
            this.parent.addData(caption, value);
            return this;
        }

        @Override
        public synchronized <T> Item addData(String caption, Func<T> func) {
            this.parent.addData(caption, func);
            return this;
        }

        @Override
        public synchronized <T> Item addData(String caption, String value, Func<T> func) {
            this.parent.addData(caption, value, func);
            return this;
        }
    }

    private transient RobotDashboard dashboard;

    protected long timestamp;
    protected List<LineImpl> lines;
    protected LogImpl log;
    protected transient boolean autoClear;
    protected transient int msTransmissionInterval;
    protected String captionValueSeparator;
    protected String itemSeparator;
    private transient ExecutorService updateExecutorService;
    private TelemetryUpdateThread updateThread;

    public DashboardTelemetry(RobotDashboard dashboard) {
        this.dashboard = dashboard;
        updateThread = new TelemetryUpdateThread();
        updateExecutorService = ThreadPool.newSingleThreadExecutor("dashboard telemetry update");
        updateExecutorService.submit(updateThread);
        resetTelemetryForOpMode();
    }

    public synchronized void resetTelemetryForOpMode() {
        this.lines = new ArrayList<>();
        this.log = new LogImpl();
        this.autoClear = true;
        this.msTransmissionInterval = 250;
        this.captionValueSeparator = " : ";
        this.itemSeparator = " | ";
    }

    @Override
    public synchronized Item addData(String caption, String value, Object... objects) {
        LineImpl line = new LineImpl();
        Item item = line.addData(caption, value, objects);
        this.lines.add(line);
        return item;
    }

    @Override
    public synchronized Item addData(String caption, Object value) {
        LineImpl line = new LineImpl();
        Item item = line.addData(caption, value);
        this.lines.add(line);
        return item;
    }

    @Override
    public synchronized <T> Item addData(String caption, Func<T> func) {
        LineImpl line = new LineImpl();
        Item item = line.addData(caption, func);
        this.lines.add(line);
        return item;
    }

    @Override
    public synchronized <T> Item addData(String caption, String value, Func<T> func) {
        LineImpl line = new LineImpl();
        Item item = line.addData(caption, value, func);
        this.lines.add(line);
        return item;
    }

    @Override
    public synchronized boolean removeItem(Item item) {
        for (Line line : lines) {
            if (line instanceof LineImpl && ((LineImpl) line).items.remove(item)) {
                return true;
            }
        }
        return false;
    }

    @Override
    public synchronized void clear() {
        int i = 0;
        while (i < this.lines.size()) {
            LineImpl line = this.lines.get(i);
            boolean keepLine = false;
            int j = 0;
            while (j < line.items.size()) {
                ItemImpl item = line.items.get(j);
                if (item.isRetained()) {
                    keepLine = true;
                    j++;
                } else {
                    line.items.remove(j);
                }
            }
            if (!keepLine) {
                this.lines.remove(i);
            } else {
                i++;
            }
        }
    }

    @Override
    public synchronized void clearAll() {
        this.lines.clear();
        this.log.clear();
    }

    @Override
    public synchronized Object addAction(Runnable runnable) {
        throw new UnsupportedOperationException();
    }

    @Override
    public synchronized boolean removeAction(Object action) {
        throw new UnsupportedOperationException();
    }

    @Override
    public synchronized boolean update() {
        this.timestamp = System.currentTimeMillis();
        dashboard.sendAll(new Message(MessageType.RECEIVE_TELEMETRY, this));
        if (autoClear) {
            clear();
        }
        return true;
    }

    @Override
    public synchronized Line addLine() {
        LineImpl line = new LineImpl();
        this.lines.add(line);
        return line;
    }

    @Override
    public synchronized Line addLine(String caption) {
        throw new UnsupportedOperationException();
    }

    @Override
    public synchronized boolean removeLine(Line line) {
        return this.lines.remove(line);
    }

    @Override
    public synchronized boolean isAutoClear() {
        return this.autoClear;
    }

    @Override
    public synchronized void setAutoClear(boolean autoClear) {
        this.autoClear = autoClear;
    }

    @Override
    public synchronized int getMsTransmissionInterval() {
        return msTransmissionInterval;
    }

    @Override
    public synchronized void setMsTransmissionInterval(int msTransmissionInterval) {
        this.msTransmissionInterval = msTransmissionInterval;
    }

    @Override
    public synchronized String getItemSeparator() {
        return this.itemSeparator;
    }

    @Override
    public synchronized void setItemSeparator(String itemSeparator) {
        this.itemSeparator = itemSeparator;
    }

    @Override
    public synchronized String getCaptionValueSeparator() {
        return this.captionValueSeparator;
    }

    @Override
    public synchronized void setCaptionValueSeparator(String captionValueSeparator) {
        this.captionValueSeparator = captionValueSeparator;
    }

    @Override
    public synchronized Log log() {
        return this.log;
    }

    public void stop() {
        if (updateExecutorService != null) {
            updateExecutorService.shutdownNow();
            updateExecutorService = null;
            updateThread = null;
        }
    }
}
