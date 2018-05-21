package com.acmerobotics.cameraoverlay


import android.accessibilityservice.AccessibilityService
import android.app.ActivityManager
import android.content.Context
import android.graphics.Bitmap
import android.graphics.Color
import android.graphics.PixelFormat
import android.os.Handler
import android.os.Looper
import android.os.Message
import android.util.TypedValue
import android.view.Gravity
import android.view.View
import android.view.WindowManager
import android.view.accessibility.AccessibilityEvent
import android.widget.ImageView
import android.widget.LinearLayout
import android.widget.RelativeLayout
import android.widget.TextView

class CameraOverlayService : AccessibilityService(), CameraStreamClient.Listener {
    companion object {
        const val OVERLAY_PACKAGE = "com.acmerobotics.cameraoverlay"
        const val OVERLAY_MAIN_ACTIVITY = "$OVERLAY_PACKAGE.MainActivity"
        const val DS_PACKAGE = "com.qualcomm.ftcdriverstation"
        const val DS_MAIN_ACTIVITY = "$DS_PACKAGE.FtcDriverStationActivity"
        const val SYSTEM_PACKAGE_PREFIX = "android"
    }

    private lateinit var streamClient: CameraStreamClient
    private lateinit var imageView: ImageView
    private lateinit var cameraButtonLayout: RelativeLayout
    private lateinit var windowManager: WindowManager
    private lateinit var activityManager: ActivityManager
    private lateinit var handler: Handler
    private lateinit var visibilityIcon: ImageView
    private var active = false
    private var streamVisible = false
    private var scale: Float = 0f

    override fun onInterrupt() {

    }

    override fun onAccessibilityEvent(event: AccessibilityEvent) {
        if (event.eventType == AccessibilityEvent.TYPE_WINDOW_STATE_CHANGED) {
            if (event.className == DS_MAIN_ACTIVITY ||
                    (event.packageName == OVERLAY_PACKAGE && event.className != OVERLAY_MAIN_ACTIVITY)) {
                if (!active) {
                    addOverlay();
                    streamClient = CameraStreamClient(handler)
                    streamClient.setListener(this)
                    streamClient.start()
                    active = true;
                }
            } else if (active && !event.className.startsWith(SYSTEM_PACKAGE_PREFIX)) {
                removeOverlay();
                streamClient.stop()
                active = false;
            }
        }
    }

    override fun onConnect() {
        handler.post { visibilityIcon.visibility = View.VISIBLE }
    }

    override fun onDisconnect() {
        handler.post {
            visibilityIcon.visibility = View.INVISIBLE
            visibilityIcon.setImageResource(R.drawable.ic_visibility_white_24dp)
            imageView.visibility = View.INVISIBLE
            streamVisible = false
        }
    }

    private fun dpToPx(dp: Float): Int = (dp * scale + 0.5f).toInt()

    override fun onCreate() {
        super.onCreate()

        scale = resources.displayMetrics.density

        activityManager = getSystemService(ActivityManager::class.java)
        windowManager = getSystemService(Context.WINDOW_SERVICE) as WindowManager

        handler = object: Handler(Looper.getMainLooper()) {
            override fun handleMessage(msg: Message) {
                imageView.setImageBitmap(msg.obj as Bitmap)
            }
        }
    }

    override fun onDestroy() {
        super.onDestroy()

        if (active) {
            removeOverlay()
            streamClient.stop()
        }
    }

    private fun addCameraButton() {
        val textView = TextView(this)
        val textViewParams = LinearLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT)
        textView.layoutParams = textViewParams
        textView.setText(R.string.cameraText)
        textView.setTextSize(TypedValue.COMPLEX_UNIT_SP, 10f)
        textView.setTextColor(Color.WHITE)

        streamVisible = false;

        visibilityIcon = ImageView(this)
        val imageSize = dpToPx(25f)
        val imagePadding = dpToPx(2f)
        val imageViewParams = LinearLayout.LayoutParams(imageSize, imageSize)
        visibilityIcon.layoutParams = imageViewParams
        visibilityIcon.setPadding(imagePadding, imagePadding, imagePadding, imagePadding)
        visibilityIcon.setImageResource(R.drawable.ic_visibility_white_24dp)
        visibilityIcon.visibility = View.INVISIBLE

        val linearLayout = LinearLayout(this)
        linearLayout.gravity = Gravity.CENTER_HORIZONTAL
        linearLayout.orientation = LinearLayout.VERTICAL
        val linearLayoutParams = RelativeLayout.LayoutParams(LinearLayout.LayoutParams.WRAP_CONTENT, LinearLayout.LayoutParams.WRAP_CONTENT)
        linearLayoutParams.addRule(RelativeLayout.ALIGN_PARENT_BOTTOM)
        linearLayout.layoutParams = linearLayoutParams
        linearLayout.addView(visibilityIcon)
        linearLayout.addView(textView)

        linearLayout.setOnClickListener {
            if (streamVisible) {
                visibilityIcon.setImageResource(R.drawable.ic_visibility_white_24dp)
                imageView.visibility = View.INVISIBLE
                streamVisible = false
            } else {
                visibilityIcon.setImageResource(R.drawable.ic_visibility_off_white_24dp)
                imageView.visibility = View.VISIBLE
                streamVisible = true
            }
        }

        cameraButtonLayout = RelativeLayout(this)
        cameraButtonLayout.setPadding(0, 0, 0, dpToPx(12f))
        cameraButtonLayout.addView(linearLayout)

        val params = WindowManager.LayoutParams(
                WindowManager.LayoutParams.WRAP_CONTENT,
                WindowManager.LayoutParams.WRAP_CONTENT,
                WindowManager.LayoutParams.TYPE_ACCESSIBILITY_OVERLAY,
                WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE or WindowManager.LayoutParams.FLAG_NOT_TOUCH_MODAL,
                PixelFormat.TRANSLUCENT)
        params.gravity = Gravity.CENTER or Gravity.TOP
        params.height = dpToPx(80f);

        windowManager.addView(cameraButtonLayout, params)
    }

    private fun addImageView() {
        windowManager = getSystemService(Context.WINDOW_SERVICE) as WindowManager

        imageView = ImageView(this)
        imageView.visibility = View.INVISIBLE

        val params = WindowManager.LayoutParams(
                WindowManager.LayoutParams.WRAP_CONTENT,
                dpToPx(160f),
                WindowManager.LayoutParams.TYPE_ACCESSIBILITY_OVERLAY,
                WindowManager.LayoutParams.FLAG_NOT_FOCUSABLE or WindowManager.LayoutParams.FLAG_NOT_TOUCH_MODAL,
                PixelFormat.TRANSLUCENT)
        params.gravity = Gravity.CENTER_HORIZONTAL or Gravity.TOP
        params.y = dpToPx(80f)

        windowManager.addView(imageView, params)
    }

    private fun removeCameraButton() {
        windowManager.removeView(cameraButtonLayout)
    }

    private fun removeImageView() {
        windowManager.removeView(imageView)
    }

    private fun addOverlay() {
        addCameraButton()
        addImageView()
    }

    private fun removeOverlay() {
        removeCameraButton()
        removeImageView()
    }
}