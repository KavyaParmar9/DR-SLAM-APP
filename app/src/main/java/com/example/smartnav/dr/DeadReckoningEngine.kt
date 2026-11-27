package com.example.smartnav.dr

import android.content.Context
import android.hardware.Sensor
import android.hardware.SensorEvent
import android.hardware.SensorEventListener
import android.hardware.SensorManager
import com.example.smartnav.data.Pose2D
import com.example.smartnav.sensors.OrientationProvider
import kotlin.math.cos
import kotlin.math.sin
import kotlin.math.sqrt

/**
 * Pedestrian Dead Reckoning (PDR) engine.
 *
 * This class fuses acceleration (for step detection) and rotation vector (for heading) sensors
 * to maintain a simple planar pose estimate. The math mirrors what is often taught in IE415:
 * each detected step advances the position by a nominal step length and heading determines
 * the direction of that step. No external references are used, so drift naturally accumulates.
 */
class DeadReckoningEngine(
    context: Context,
    private val stepLengthMeters: Float = 0.75f,
    private val listener: (Pose2D) -> Unit
) : SensorEventListener {

    private val sensorManager: SensorManager = requireNotNull(context.getSystemService(SensorManager::class.java))
    private val accelSensor: Sensor? = sensorManager.getDefaultSensor(Sensor.TYPE_LINEAR_ACCELERATION)
    private val orientationProvider = OrientationProvider(sensorManager)

    private var isRunning = false
    private var pose = Pose2D(0f, 0f, 0f)
    private var lastStepTimestampNs = 0L
    private var highPassAccel = 0f

    private val stepThreshold = 1.0f
    private val minStepIntervalNs = 400_000_000L // 0.4 s
    private val alpha = 0.8f // High-pass filter coefficient

    fun start() {
        if (isRunning) return
        accelSensor?.let {
            sensorManager.registerListener(this, it, SensorManager.SENSOR_DELAY_GAME)
        }
        orientationProvider.start()
        isRunning = true
    }

    fun stop() {
        if (!isRunning) return
        sensorManager.unregisterListener(this)
        orientationProvider.stop()
        isRunning = false
    }

    fun reset() {
        pose = Pose2D(0f, 0f, 0f)
        lastStepTimestampNs = 0L
        listener.invoke(pose)
    }

    override fun onSensorChanged(event: SensorEvent) {
        if (event.sensor.type == Sensor.TYPE_LINEAR_ACCELERATION) {
            processAcceleration(event.values, event.timestamp)
        }
    }

    private fun processAcceleration(values: FloatArray, timestampNs: Long) {
        // Magnitude of linear acceleration vector ~ how "strong" the step impact is.
        val magnitude = sqrt(values[0] * values[0] + values[1] * values[1] + values[2] * values[2])
        // High-pass filter removes slow drift/bias so only sharp peaks remain.
        highPassAccel = alpha * (highPassAccel + magnitude - lastMagnitude) + (1 - alpha) * magnitude
        lastMagnitude = magnitude

        if (highPassAccel > stepThreshold && timestampNs - lastStepTimestampNs > minStepIntervalNs) {
            // Step detected! advance the planar pose in the current heading direction.
            lastStepTimestampNs = timestampNs
            advancePose()
        }
    }

    private var lastMagnitude = 0f

    private fun advancePose() {
        val heading = orientationProvider.currentHeadingRad
        // Classic PDR integration: displace by L * [cos(theta), sin(theta)].
        val newX = pose.x + stepLengthMeters * cos(heading)
        val newY = pose.y + stepLengthMeters * sin(heading)
        pose = Pose2D(newX, newY, heading)
        listener.invoke(pose)
    }

    override fun onAccuracyChanged(sensor: Sensor?, accuracy: Int) = Unit
}

