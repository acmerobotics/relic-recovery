package com.acmerobotics.library.dashboard;

import android.content.Context;
import android.content.SharedPreferences;
import android.util.Log;

import com.acmerobotics.library.dashboard.canvas.Canvas;
import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.dashboard.config.Configuration;
import com.acmerobotics.library.dashboard.config.Option;
import com.acmerobotics.library.dashboard.message.Message;
import com.acmerobotics.library.dashboard.message.MessageDeserializer;
import com.acmerobotics.library.dashboard.message.MessageType;
import com.acmerobotics.library.dashboard.telemetry.DashboardTelemetry;
import com.acmerobotics.library.dashboard.util.ClassFilter;
import com.acmerobotics.library.dashboard.util.ClasspathScanner;
import com.google.gson.Gson;
import com.google.gson.GsonBuilder;
import com.google.gson.JsonElement;
import com.qualcomm.robotcore.eventloop.EventLoop;

import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.io.IOException;
import java.util.ArrayList;
import java.util.List;

public class RobotDashboard {
    public static final String TAG = "RobotDashboard";
	public static final String CONFIG_PREFS = "config";

	public static final Gson GSON = new GsonBuilder()
            .registerTypeAdapter(Message.class, new MessageDeserializer())
			.create();

	private static RobotDashboard dashboard;

	// TODO I'm sure there's a better way to make a static singleton
	public static RobotDashboard open(Context ctx, EventLoop eventLoop) {
		// the eventLoop can be used to get the op mode manager and monitor op mode activity
		dashboard = new RobotDashboard(ctx);
		return dashboard;
	}

	public static RobotDashboard getInstance() {
		return dashboard;
	};

	private DashboardTelemetry telemetry;
	private SharedPreferences prefs;
	private List<RobotWebSocket> sockets;
	private RobotWebSocketServer server;
	private Configuration configuration;
	private Canvas fieldOverlay;
	
	private RobotDashboard(Context ctx) {
		prefs = ctx.getSharedPreferences(CONFIG_PREFS, Context.MODE_PRIVATE);
		sockets = new ArrayList<>();
		fieldOverlay = new Canvas();
		configuration = new Configuration();
		telemetry = new DashboardTelemetry(this);

        ClasspathScanner scanner = new ClasspathScanner(new ClassFilter() {
            @Override
            public boolean shouldProcessClass(String className) {
                return className.startsWith("com.acmerobotics");
            }

            @Override
            public void processClass(Class clazz) {
                if (clazz.isAnnotationPresent(Config.class)) {
                	Log.i(TAG, String.format("Found config class %s", clazz.getCanonicalName()));
					configuration.addOptionsFromClass(clazz);
                }
            }
        });
        scanner.scanClasspath();

		server = new RobotWebSocketServer(this);
		try {
			server.start();
		} catch (IOException e) {
			e.printStackTrace();
		}
	}

	public void addOption(String name, Option option) {
		configuration.addOption(name, option);
	}

	public void resetConfigurationForOpMode() {
		telemetry.resetTelemetryForOpMode();
	}

	public void registerConfigClass(Class<?> configClass, String name) {
		configuration.addOptionsFromClass(configClass, name);
	    sendAll(new Message(MessageType.RECEIVE_CONFIG, configuration));
    }

    public void registerConfigClass(Class<?> configClass) {
		configuration.addOptionsFromClass(configClass);
		sendAll(new Message(MessageType.RECEIVE_CONFIG, configuration));
	}

    public Telemetry getTelemetry() {
	    return telemetry;
    }

    public Canvas getFieldOverlay() {
        return fieldOverlay;
    }

    public void drawOverlay() {
        sendAll(new Message(MessageType.RECEIVE_FIELD_OVERLAY, fieldOverlay));
        fieldOverlay.clear();
    }

    public JsonElement getConfigSchemaJson() {
	    return configuration.getJsonSchema();
    }

    public JsonElement getConfigJson() {
		return configuration.getJson();
	}

	public synchronized void sendAll(Message message) {
		for (RobotWebSocket ws : sockets) {
			ws.send(message);
		}
	}

	public synchronized void addSocket(RobotWebSocket socket) {
		sockets.add(socket);
		socket.send(new Message(MessageType.RECEIVE_CONFIG_SCHEMA, getConfigSchemaJson()));
		socket.send(new Message(MessageType.RECEIVE_CONFIG, getConfigJson()));
	}

	public synchronized void removeSocket(RobotWebSocket socket) {
		sockets.remove(socket);
	}

	public synchronized void onMessage(RobotWebSocket socket, Message msg) {
        switch(msg.getType()) {
            case GET_CONFIG: {
                socket.send(new Message(MessageType.RECEIVE_CONFIG, getConfigJson()));
                break;
            }
			case SAVE_CONFIG: {
            	configuration.updateJson((JsonElement) msg.getData());
                break;
            }
            default:
                Log.w(TAG, String.format("unknown message recv'd: '%s'", msg.getType()));
                Log.w(TAG, msg.toString());
                break;
        }
	}

	public void stop() {
		server.stop();
		dashboard = null;
	}
}
