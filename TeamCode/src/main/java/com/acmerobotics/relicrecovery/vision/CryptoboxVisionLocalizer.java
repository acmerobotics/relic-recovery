package com.acmerobotics.relicrecovery.vision;

/**
 * @author Ryan
 */

public class CryptoboxVisionLocalizer {


    public static int HORIZONTAL_OFFSET = -1;
    public static int DISTANCE_OFFSET = 22;

//    public Pose2d getPose() {
//
//        // pose-based rail hallucination
//        if (poseEstimator != null && rails.size() >= 2) {
//            // TODO: not fully implemented yet
//            Vector2d nearBlueCryptobox = new Vector2d(12, -72);
//            Vector2d estimatedPos = poseEstimator.getPose().pos();
//            if (Vector2d.distance(nearBlueCryptobox, estimatedPos) < 24) {
//                //
//            }
//        }
//
//        // fancy heuristic stuff
//        if (rails.size() == 2 || rails.size() == 3) {
//            double meanRailGap = getMeanRailGap(rails);
//            if (rails.get(0) < meanRailGap) {
//                while (actualWidth - rails.get(rails.size() - 1) > meanRailGap && rails.size() < 4) {
//                    // add extra rail on the left
//                    rails.add(0, rails.get(0) - meanRailGap);
//                }
//            } else if (actualWidth - rails.get(rails.size() - 1) < meanRailGap) {
//                while (rails.get(0) > meanRailGap && rails.size() < 4) {
//                    // add extra rail on the right
//                    rails.add(rails.get(rails.size() - 1) + meanRailGap);
//                }
//            }
//        }
//
//        // calculate the distance and horizontal offset of the cryptobox
//        double distance = Double.NaN, offsetX = Double.NaN;
//        if (rails.size() > 1) {
//            double meanRailGap = getMeanRailGap(rails);
//            distance = (ACTUAL_RAIL_GAP * focalLengthPx) / meanRailGap;
//            if (rails.size() == 4) {
//                double center = (Collections.max(rails) + Collections.min(rails)) / 2.0;
//                offsetX = ((0.5 * actualWidth - center) * distance) / focalLengthPx;
//            }
//        }
//    }
}
