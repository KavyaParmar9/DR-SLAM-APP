package com.example.smartnav.data

import android.content.Context
import android.os.SystemClock
import java.io.File
import java.io.FileWriter
import java.time.Instant
import java.time.format.DateTimeFormatter
import java.util.Locale

/**
 * Stores sampled trajectories and exposes simple helpers for saving and loading CSV logs.
 */
class TrajectoryRepository {

    private val deadReckoningPoints = mutableListOf<Pose2D>()
    private val slamPoints = mutableListOf<Pose2D>()
    private val timestamps = mutableListOf<Long>()

    fun reset() {
        deadReckoningPoints.clear()
        slamPoints.clear()
        timestamps.clear()
    }

    fun addDeadReckoningPose(pose: Pose2D) {
        deadReckoningPoints += pose
        addTimestampIfMissing()
    }

    fun addSlamPose(pose: Pose2D) {
        slamPoints += pose
        addTimestampIfMissing()
    }

    private fun addTimestampIfMissing() {
        if (timestamps.size < maxOf(deadReckoningPoints.size, slamPoints.size)) {
            timestamps += SystemClock.elapsedRealtime()
        }
    }

    fun getDeadReckoning(): List<Pose2D> = deadReckoningPoints.toList()

    fun getSlam(): List<Pose2D> = slamPoints.toList()

    fun computeDrift(): Float {
        if (deadReckoningPoints.isEmpty() || slamPoints.isEmpty()) return 0f
        val drPose = deadReckoningPoints.last()
        val slamPose = slamPoints.last()
        val dx = slamPose.x - drPose.x
        val dy = slamPose.y - drPose.y
        return kotlin.math.sqrt(dx * dx + dy * dy)
    }

    fun saveToCsv(context: Context): File? {
        if (deadReckoningPoints.isEmpty() && slamPoints.isEmpty()) return null
        val sessionDir = File(context.filesDir, "sessions").apply { mkdirs() }
        val timestamp = DateTimeFormatter.ISO_INSTANT.format(Instant.now())
        val file = File(sessionDir, "session_$timestamp.csv".replace(":", "-"))
        FileWriter(file).use { writer ->
            writer.appendLine("timestamp_ms,dr_x,dr_y,slam_x,slam_y")
            val maxSize = maxOf(deadReckoningPoints.size, slamPoints.size)
            for (i in 0 until maxSize) {
                val time = timestamps.getOrNull(i) ?: timestamps.lastOrNull() ?: 0L
                val dr = deadReckoningPoints.getOrNull(i)
                val slam = slamPoints.getOrNull(i)
                writer.appendLine(
                    listOf(
                        time.toString(),
                        formatValue(dr?.x),
                        formatValue(dr?.y),
                        formatValue(slam?.x),
                        formatValue(slam?.y)
                    ).joinToString(",")
                )
            }
        }
        return file
    }

    fun loadLastSession(context: Context): Pair<List<Pose2D>, List<Pose2D>>? {
        val sessionDir = File(context.filesDir, "sessions")
        if (!sessionDir.exists()) return null
        val lastFile = sessionDir.listFiles()?.maxByOrNull { it.lastModified() } ?: return null
        val drList = mutableListOf<Pose2D>()
        val slamList = mutableListOf<Pose2D>()
        lastFile.forEachLine { line ->
            if (line.startsWith("timestamp") || line.isBlank()) return@forEachLine
            val parts = line.split(",")
            if (parts.size < 5) return@forEachLine
            val drX = parts[1].toFloatOrNull()
            val drY = parts[2].toFloatOrNull()
            val slamX = parts[3].toFloatOrNull()
            val slamY = parts[4].toFloatOrNull()
            if (drX != null && drY != null) {
                drList += Pose2D(drX, drY, 0f)
            }
            if (slamX != null && slamY != null) {
                slamList += Pose2D(slamX, slamY, 0f)
            }
        }
        replaceTrajectories(drList, slamList)
        return drList to slamList
    }

    private fun replaceTrajectories(dr: List<Pose2D>, slam: List<Pose2D>) {
        deadReckoningPoints.apply {
            clear()
            addAll(dr)
        }
        slamPoints.apply {
            clear()
            addAll(slam)
        }
        timestamps.apply {
            clear()
            val count = maxOf(dr.size, slam.size)
            repeat(count) { add(SystemClock.elapsedRealtime()) }
        }
    }

    private fun formatValue(value: Float?): String =
        value?.let { String.format(Locale.US, "%.3f", it) } ?: ""
}
