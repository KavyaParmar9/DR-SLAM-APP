package com.example.smartnav.data

/**
 * Simple planar pose used for both dead reckoning and SLAM.
 *
 * @property x position along the device's initial X axis in meters.
 * @property y position along the device's initial Y axis in meters.
 * @property theta heading (yaw) in radians relative to the initial orientation.
 */
data class Pose2D(
    val x: Float,
    val y: Float,
    val theta: Float
)


