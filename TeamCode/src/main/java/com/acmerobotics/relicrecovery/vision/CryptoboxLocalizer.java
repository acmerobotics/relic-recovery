package com.acmerobotics.relicrecovery.drive;

import android.util.Log;

import com.acmerobotics.library.dashboard.config.Config;
import com.acmerobotics.library.localization.Pose2d;
import com.acmerobotics.library.localization.Vector2d;
import com.acmerobotics.relicrecovery.configuration.AllianceColor;
import com.acmerobotics.relicrecovery.configuration.Cryptobox;
import com.acmerobotics.relicrecovery.subsystems.MecanumDrive;
import com.acmerobotics.relicrecovery.vision.CryptoboxTracker;
import com.acmerobotics.relicrecovery.vision.VisionCamera;

import java.util.ArrayList;
import java.util.Collections;
import java.util.List;

import static com.acmerobotics.relicrecovery.vision.CryptoboxTracker.ACTUAL_RAIL_GAP;
import static com.acmerobotics.relicrecovery.vision.OldCryptoboxTracker.getMeanRailGap;

/**
 * @author Ryan
 */

@Config
public class CryptoboxLocalizer implements CryptoboxTracker.Listener {
    public static int HORIZONTAL_OFFSET = -3;
    public static int DISTANCE_OFFSET = -3;

    public interface Listener {
        void onPositionUpdate(Vector2d position, double timestamp);
    }

    private CryptoboxTracker tracker;
    private MecanumDrive drive;
    private Vector2d latestEstimatedPos;
    private List<Listener> listeners;
    private VisionCamera.Properties cameraProperties;

    public CryptoboxLocalizer(CryptoboxTracker tracker, VisionCamera.Properties cameraProperties, MecanumDrive drive) {
        this.tracker = tracker;
        this.cameraProperties = cameraProperties;
        this.drive = drive;
        this.listeners = new ArrayList<>();
    }

    private Vector2d getPosFromRails(List<Double> rails) {
        double resizedWidth = tracker.getResizedWidth();
        double focalLengthPx = cameraProperties.getHorizontalFocalLengthPx(resizedWidth);

        // TODO: maybe refactor into some functions
        // generally make less awful
        double distance = Double.NaN, offset = Double.NaN;
        if (rails.size() > 1) {
            double meanRailGap = getMeanRailGap(rails);
            distance = (ACTUAL_RAIL_GAP * focalLengthPx) / meanRailGap;
            if (rails.size() == 4) {
                double center = (Collections.max(rails) + Collections.min(rails)) / 2.0;
                offset = ((0.5 * resizedWidth - center) * distance) / focalLengthPx;
            }
        }
        return new Vector2d(distance - DISTANCE_OFFSET, offset - HORIZONTAL_OFFSET);
    }

    public static Vector2d getFieldPositionFromCryptoRelativePosition(Cryptobox cryptobox, Vector2d cryptoRelPos) {
        Vector2d cryptoPos = cryptobox.getPose().pos();
        switch (cryptobox) {
            case NEAR_BLUE:
                return new Vector2d(cryptoPos.x() - cryptoRelPos.y(), cryptoPos.y() + cryptoRelPos.x());
            case NEAR_RED:
                return new Vector2d(cryptoPos.x() - cryptoRelPos.y(), cryptoPos.y() - cryptoRelPos.x());
            case FAR_BLUE:
            case FAR_RED:
                // intentional fall-through
                return cryptoPos.added(cryptoRelPos);
        }
        throw new RuntimeException("Invalid cryptobox!");
    }

    public Cryptobox getClosestCryptobox() {
        boolean facingFarWall = Math.abs(drive.getHeading()) < Math.PI / 4;
        if (tracker.getColor() == AllianceColor.RED) {
            return facingFarWall ? Cryptobox.FAR_RED : Cryptobox.NEAR_RED;
        } else {
            return facingFarWall ? Cryptobox.FAR_BLUE : Cryptobox.NEAR_BLUE;
        }
    }

    public synchronized Vector2d getLatestEstimatedPosition() {
        return latestEstimatedPos;
    }

    @Override
    public synchronized void onCryptoboxDetection(List<Double> rails, double timestamp) {
        if (rails.size() < 2 || getMeanRailGap(rails) < 25) {
            latestEstimatedPos = new Vector2d(Double.NaN, Double.NaN);
        } else {
            Pose2d robotPose = drive.getEstimatedPose();
            Cryptobox cryptobox = getClosestCryptobox();
            if (rails.size() == 4) {
                // we're good
                Log.i("CryptoboxLocalizer", "rails: " + rails);
                Log.i("CryptoboxLocalizer", "relative position: " + getPosFromRails(rails));
                latestEstimatedPos = getFieldPositionFromCryptoRelativePosition(
                        cryptobox, getPosFromRails(rails));
                Log.i("CryptoboxLocalizer", "cryptobox: " + cryptobox);
                Log.i("CryptoboxLocalizer", "final position: " + latestEstimatedPos);
            } else if (rails.size() < 2 || rails.size() > 4) {
                // uh-oh
                latestEstimatedPos = new Vector2d(Double.NaN, Double.NaN);
            } else {
                // fancy stuff
                Log.i("CryptoboxLocalizer", "current " + robotPose);
                double meanRailGap = getMeanRailGap(rails);
                int numEstimatedRails = 4 - rails.size();
                double bestError = Double.MAX_VALUE;
                Vector2d bestPos = new Vector2d(Double.NaN, Double.NaN);
                for (int leftRailsToAdd = 0; leftRailsToAdd <= numEstimatedRails; leftRailsToAdd++) {
                    List<Double> railsCopy = new ArrayList<>(rails);
                    int rightRailsToAdd = numEstimatedRails - leftRailsToAdd;
                    for (int i = 0; i < leftRailsToAdd; i++) {
                        // add extra rail on the left
                        railsCopy.add(0, railsCopy.get(0) - meanRailGap);
                    }
                    for (int i = 0; i < rightRailsToAdd; i++) {
                        // add extra rail on the right
                        railsCopy.add(railsCopy.get(railsCopy.size() - 1) + meanRailGap);
                    }
                    Vector2d pos = getFieldPositionFromCryptoRelativePosition(
                            cryptobox, getPosFromRails(railsCopy));
                    Log.i("CryptoboxLocalizer", "considering " + pos);
                    double error = robotPose.pos().added(pos.negated()).norm();
                    if (error < bestError) {
                        bestError = error;
                        bestPos = pos;
                    }
                }
                Log.i("CryptoboxLocalizer", "chose " + bestPos);
                latestEstimatedPos = bestPos;
            }
        }

        for (Listener listener : listeners) {
            listener.onPositionUpdate(latestEstimatedPos, timestamp);
        }
    }

    public synchronized void addListener(Listener listener) {
        listeners.add(listener);
    }

    public synchronized void removeListener(Listener listener) {
        listeners.remove(listener);
    }
}
