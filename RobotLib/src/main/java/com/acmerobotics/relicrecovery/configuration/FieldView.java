package com.acmerobotics.relicrecovery.configuration;

import android.content.Context;
import android.graphics.Bitmap;
import android.graphics.BitmapFactory;
import android.graphics.Canvas;
import android.graphics.Color;
import android.graphics.Paint;
import android.graphics.Rect;
import android.support.annotation.Nullable;
import android.util.AttributeSet;
import android.view.View;

import com.acmerobotics.library.R;
import com.acmerobotics.library.localization.Pose2d;

/**
 * Created by ryanbrott on 11/12/17.
 */

public class FieldView extends View {
    private Bitmap fieldBitmap;
    private OpModeConfiguration configuration;
    private Paint paint;

    public FieldView(Context context) {
        super(context);
    }

    public FieldView(Context context, @Nullable AttributeSet attrs) {
        super(context, attrs);
    }

    public FieldView(Context context, @Nullable AttributeSet attrs, int defStyleAttr) {
        super(context, attrs, defStyleAttr);
    }

    private static double scale(double value, double fromStart, double fromEnd, double toStart, double toEnd) {
        return toStart + ((toEnd - toStart) * (value - fromStart) / (fromEnd - fromStart));
    }

    @Override
    protected void onDraw(Canvas canvas) {
        if (fieldBitmap == null) {
            fieldBitmap = BitmapFactory.decodeResource(getResources(), R.drawable.field);
        }
        canvas.drawBitmap(fieldBitmap, new Rect(0, 0, fieldBitmap.getWidth(), fieldBitmap.getHeight()),
                new Rect(0, 0, canvas.getWidth(), canvas.getHeight()), null);

        if (paint == null) {
            paint = new Paint();
            paint.setColor(Color.GREEN);
            paint.setStyle(Paint.Style.FILL);
        }

        if (configuration != null) {
            BalancingStone balancingStone = configuration.getBalancingStone();
            Pose2d pose = balancingStone.getPose();
            canvas.drawCircle((float) scale(pose.y(), 72, -72, 0, canvas.getWidth()),
                    (float) scale(pose.x(), 72, -72, 0, canvas.getHeight()), 10, paint);
        }
    }

    public void setConfiguration(OpModeConfiguration configuration) {
        this.configuration = configuration;
    }
}
