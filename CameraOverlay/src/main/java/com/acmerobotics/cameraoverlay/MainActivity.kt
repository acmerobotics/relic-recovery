package com.acmerobotics.cameraoverlay

import android.content.Intent
import android.content.pm.PackageManager
import android.net.Uri
import android.os.Bundle
import android.provider.Settings
import android.support.v7.app.AppCompatActivity
import android.text.TextUtils
import android.view.View
import android.widget.Button
import android.widget.ImageView

class MainActivity : AppCompatActivity() {
    private lateinit var overlayStatus: ImageView
    private lateinit var accessibilityStatus: ImageView
    private lateinit var dsStatus: ImageView

    private lateinit var overlayOpen: ImageView
    private lateinit var accessibilityOpen: ImageView
    private lateinit var dsOpen: ImageView

    private lateinit var openButton: Button

    private lateinit var serviceIntent: Intent

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        setContentView(R.layout.activity_main)

        overlayStatus = findViewById(R.id.overlayStatus)
        overlayOpen = findViewById(R.id.overlayOpen)
        accessibilityStatus = findViewById(R.id.accessibilityStatus)
        accessibilityOpen = findViewById(R.id.accessibilityOpen)
        dsStatus = findViewById(R.id.dsStatus)
        dsOpen = findViewById(R.id.dsOpen)

        overlayOpen.setOnClickListener {
            val intent = Intent(Settings.ACTION_MANAGE_OVERLAY_PERMISSION,
                    Uri.parse("package:" + packageName))
            startActivity(intent)
        };

        accessibilityOpen.setOnClickListener {
            val intent = Intent(Settings.ACTION_ACCESSIBILITY_SETTINGS)
            startActivity(intent)
        }

        dsOpen.setOnClickListener {
            val intent = Intent(Intent.ACTION_VIEW)
            intent.data = Uri.parse("market://details?id=${CameraOverlayService.DS_PACKAGE}")
            startActivity(intent)
        }

        serviceIntent = Intent(this, CameraOverlayService::class.java)

        openButton = findViewById<Button>(R.id.openButton)
        openButton.setOnClickListener { openDs() }
    }

    override fun onStart() {
        super.onStart()

        val canDrawOverlays = Settings.canDrawOverlays(this)
        val hasAccessibilityPermission = hasAccessibilityPermission()
        val dsInstalled = isDsInstalled()

        if (canDrawOverlays) {
            overlayStatus.setImageResource(R.drawable.ic_done_green_24dp)
            overlayOpen.visibility = View.INVISIBLE
        } else {
            overlayStatus.setImageResource(R.drawable.ic_clear_red_24dp)
            overlayOpen.visibility = View.VISIBLE
        }

        if (hasAccessibilityPermission) {
            accessibilityStatus.setImageResource(R.drawable.ic_done_green_24dp)
            accessibilityOpen.visibility = View.INVISIBLE
        } else {
            accessibilityStatus.setImageResource(R.drawable.ic_clear_red_24dp)
            accessibilityOpen.visibility = View.VISIBLE
        }

        if (dsInstalled) {
            dsStatus.setImageResource(R.drawable.ic_done_green_24dp)
            dsOpen.visibility = View.INVISIBLE
        } else {
            dsStatus.setImageResource(R.drawable.ic_clear_red_24dp)
            dsOpen.visibility = View.VISIBLE
        }

        if (canDrawOverlays && hasAccessibilityPermission && dsInstalled) {
            openButton.isEnabled = true
        }
    }

    private fun hasAccessibilityPermission(): Boolean {
        val service = packageName + "/" + CameraOverlayService::class.java.canonicalName
        val accessibilityEnabled = Settings.Secure.getInt(
                applicationContext.contentResolver,
                android.provider.Settings.Secure.ACCESSIBILITY_ENABLED)

        val colonSplitter = TextUtils.SimpleStringSplitter(':')

        if (accessibilityEnabled == 1) {
            val settingValue = Settings.Secure.getString(
                    applicationContext.contentResolver,
                    Settings.Secure.ENABLED_ACCESSIBILITY_SERVICES)
            if (settingValue != null) {
                colonSplitter.setString(settingValue)
                while (colonSplitter.hasNext()) {
                    val accessibilityService = colonSplitter.next()

                    if (accessibilityService.equals(service, ignoreCase = true)) {
                        return true
                    }
                }
            }
        }

        return false
    }

    private fun isDsInstalled(): Boolean {
        return try {
            packageManager.getPackageInfo(CameraOverlayService.DS_PACKAGE, 0)
            true
        } catch (e: PackageManager.NameNotFoundException) {
            false
        }
    }

    private fun openDs() {
        val launchIntent = packageManager.getLaunchIntentForPackage(CameraOverlayService.DS_PACKAGE)
        if (launchIntent != null) {
            startActivity(launchIntent)
        }
    }

    override fun onDestroy() {
        super.onDestroy()

        stopService(serviceIntent)
    }
}