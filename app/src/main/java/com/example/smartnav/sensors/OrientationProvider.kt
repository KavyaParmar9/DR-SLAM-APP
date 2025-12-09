package com.example.smartnav.sensors

import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import java.util.concurrent.atomic.AtomicBoolean

/**
 * Converts the rotation vector sensor into a yaw/heading estimate.
 *
 * Heading is expressed in radians where 0 = device facing the positive X axis at start-up,
 * positive angles rotate counter-clockwise.
 */
class OrientationProvider(
    private val sensorManager: SensorManager
) : SensorEventListener {

    private val rotationSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_ROTATION_VECTOR)
    private val running = AtomicBoolean(false)

    private val rotationMatrix = FloatArray(9)
    private val orientationAngles = FloatArray(3)

    @Volatile
    var currentHeadingRad: Float = 0f
        private set

    fun start() {
        if (running.getAndSet(true)) return
        rotationSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
    }

    fun stop() {
        if (!running.getAndSet(false)) return
        sensorManager.unregisterListener(this, rotationSensor)
    }

    override fun onSensorChanged(event: android.hardware.SensorEvent) {
        if (event.sensor.type != Sensor.TYPE_ROTATION_VECTOR) return
        SensorManager.getRotationMatrixFromVector(rotationMatrix, event.values)
        SensorManager.getOrientation(rotationMatrix, orientationAngles)
        currentHeadingRad = orientationAngles[0]
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}


