package com.example.smartnav.utils

import kotlin.math.atan2

/**
 * Shared math helpers for pose conversions.
 */
object MathUtils {

    /**
     * Converts quaternion components into yaw (heading) in radians.
     */
    fun quaternionToYaw(qx: Float, qy: Float, qz: Float, qw: Float): Float {
        val sinyCosp = 2f * (qw * qz + qx * qy)
        val cosyCosp = 1f - 2f * (qy * qy + qz * qz)
        return atan2(sinyCosp, cosyCosp)
    }
}


