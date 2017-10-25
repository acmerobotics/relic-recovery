package com.acmerobotics.library.dashboard.canvas;

import com.google.gson.annotations.SerializedName;

/**
 * Created by ryanbrott on 8/4/17.
 */

public abstract class CanvasOp {
    public enum Type {
        @SerializedName("circle")
        CIRCLE,

        @SerializedName("polygon")
        POLYGON,

        @SerializedName("polyline")
        POLYLINE,

        @SerializedName("stroke")
        STROKE,

        @SerializedName("fill")
        FILL,

        @SerializedName("strokeWidth")
        STROKE_WIDTH;
    }

    private Type type;

    public CanvasOp(Type type) {
        this.type = type;
    }
}
