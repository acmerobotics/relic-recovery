package com.acmerobotics.library.dashboard.telemetry;

import android.os.Environment;
import android.util.Log;

import java.io.BufferedReader;
import java.io.BufferedWriter;
import java.io.File;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;

public class DataFile implements AutoCloseable {

    private static final String TAG = "DataFile";

    private File file;
    private BufferedReader reader;
    private BufferedWriter writer;

    public DataFile(String filename) {
        this(filename, false);
    }

    public DataFile(String filename, boolean read) {
        openFile(filename, read);
    }

    protected void openFile(String filename, boolean read) {
        File dir = getStorageDir();
        dir.mkdirs();
        this.file = new File(dir, filename);
        try {
            this.file.createNewFile();
            if (read) {
                this.reader = new BufferedReader(new FileReader(file));
            } else {
                this.writer = new BufferedWriter(new FileWriter(file));
            }
        } catch (IOException e) {
            Log.e(TAG, "IO error while trying to open data file " + file.getPath() + "\n" + e.getMessage());
        }
    }

    public static File getStorageDir() {
        return new File(Environment.getExternalStorageDirectory(), "ACME");
    }

    public BufferedReader getReader() {
        return reader;
    }

    public BufferedWriter getWriter() {
        return writer;
    }

    public File getFile() {
        return file;
    }

    public String readLine() {
        String line = null;
        try {
            line = reader.readLine();
        } catch (IOException e) {
            Log.e(TAG, "IO error while attempting to read data from file\n" + e.getMessage());
        }
        return line;
    }

    public void write() {
        write("");
    }

    public void write(String s) {
        write(s, true);
    }

    public void write(String s, boolean newline) {
        try {
            writer.write(s + (newline ? "\r\n" : ""));
            writer.flush();
        } catch (IOException e) {
            Log.e(TAG, "IO error while attempting to write data to file\n" + e.getMessage());
        }
    }


    @Override
    public void close() {
        try {
            if (reader != null) reader.close();
            if (writer != null) writer.close();
        } catch (IOException e) {
            Log.e(TAG, "IO error while trying to close the data file\n" + e.getMessage());
        }
    }
}