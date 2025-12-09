package com.example.smartnav.slam

import android.os.Handler
import android.os.Looper
import android.util.Log
import com.example.smartnav.data.Pose2D
import com.example.smartnav.utils.MathUtils
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import io.github.sceneview.ar.ARSceneView
import kotlinx.coroutines.CoroutineScope
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.Job
import kotlinx.coroutines.delay
import kotlinx.coroutines.launch
import kotlin.math.cos
import kotlin.math.sin


class SlamTracker(
    private val arSceneView: ARSceneView,
    private val listener: (Pose2D) -> Unit
) {

    private var isTracking = false
    private var initialPose: Pose2D? = null
    private var session: Session? = null
    private val handler = Handler(Looper.getMainLooper())
    private var framePollRunnable: Runnable? = null
    private val coroutineScope = CoroutineScope(Dispatchers.Main)
    private var sessionPollJob: Job? = null
    private var frameCount = 0L
    private var lastLogTime = 0L
    private var useFallback = false
    private var lastKnownPose: Pose2D? = null
    private var lastUpdateTime = 0L
    private var lastArCorePose: Pose2D? = null
    private var arCoreVelocity: Pose2D? = null  // Track ARCore movement velocity
    private var lastArCoreUpdateTime = 0L

    companion object {
        private const val TAG = "SlamTracker"
    }

    /**
     * Start tracking SLAM poses.
     */
    fun start() {
        if (isTracking) return

        Log.d(TAG, "=== STARTING SLAM TRACKING ===")
        isTracking = true
        frameCount = 0
        lastLogTime = System.currentTimeMillis()
        lastUpdateTime = System.currentTimeMillis()
        useFallback = false

        // Start aggressive session polling
        sessionPollJob = coroutineScope.launch {
            var attempts = 0
            while (isTracking && attempts < 100) { // Try for 10 seconds
                session = getArCoreSession()
                if (session == null) {
                    delay(100)
                    attempts++
                } else {
                    Log.d(TAG, "*** ARCore session FOUND! ***")
                    configureSessionForPlaneDetection(session!!)
                    break
                }
            }
            if (session == null) {
                Log.w(TAG, "*** ARCore session NOT found - using fallback mode ***")
                useFallback = true
                // Initialize fallback with origin
                initialPose = Pose2D(0f, 0f, 0f)
                listener.invoke(Pose2D(0f, 0f, 0f))
            }
        }

        // Start frame polling immediately
        framePollRunnable = object : Runnable {
            override fun run() {
                if (!isTracking) return

                frameCount++
                val currentTime = System.currentTimeMillis()

                try {
                    // Try ARCore first
                    if (!useFallback && session == null) {
                        session = getArCoreSession()
                        if (session != null) {
                            Log.d(TAG, "*** Session found in frame loop! ***")
                            configureSessionForPlaneDetection(session!!)
                            useFallback = false
                        } else if (frameCount > 300) { // After 10 seconds, give up on ARCore
                            Log.w(TAG, "Giving up on ARCore, switching to fallback")
                            useFallback = true
                            initialPose = Pose2D(0f, 0f, 0f)
                            listener.invoke(Pose2D(0f, 0f, 0f))
                        }
                    }

                    // Method 1: Try ARCore frame
                    if (!useFallback && session != null) {
                        try {
                            var frame: Frame? = null
                            try {
                                // Try to get frame from session
                                frame = session!!.update()
                            } catch (e: Exception) {
                                // Session might be busy, skip this frame
                                if (frameCount % 100 == 0L) {
                                    Log.d(TAG, "ARCore session.update() failed: ${e.message}")
                                }
                            }

                            if (frame != null) {
                                val pose = processArCoreFrame(frame)
                                if (pose != null) {
                                    listener.invoke(pose)
                                    lastKnownPose = pose
                                    lastUpdateTime = currentTime
                                    handler.postDelayed(this, 33)
                                    return
                                }
                            }
                        } catch (e: Exception) {
                            // ARCore failed, will use fallback
                            if (frameCount % 100 == 0L) {
                                Log.d(TAG, "ARCore processing failed, using fallback: ${e.message}")
                            }
                        }
                    }

                    // Method 2: Fallback - Simulated SLAM (at least something works!)
                    if (useFallback) {
                        val pose = processSimulatedSlam(currentTime)
                        if (pose != null) {
                            listener.invoke(pose)
                            lastKnownPose = pose
                            lastUpdateTime = currentTime
                        }
                    } else if (session == null && frameCount % 200 == 0L) {
                        // Log status if ARCore session not found yet
                        Log.d(TAG, "Still waiting for ARCore session... (frame $frameCount)")
                    }

                } catch (e: Exception) {
                    if (currentTime - lastLogTime > 2000) {
                        Log.e(TAG, "Error in frame polling: ${e.message}")
                        lastLogTime = currentTime
                    }
                }

                // Continue polling at ~30 FPS
                handler.postDelayed(this, 33)
            }
        }

        handler.post(framePollRunnable!!)
        Log.d(TAG, "Frame polling started")
        
        // TEST: Immediately call listener to verify it works
        handler.postDelayed({
            Log.d(TAG, "TEST: Calling listener directly to verify UI works")
            listener.invoke(Pose2D(0f, 0f, 0f))
        }, 500)
    }

    /**
     * Process ARCore frame.
     * ARCore provides poses in meters, tracking camera position in 3D world space.
     * 
     * IMPORTANT: ARCore tracks CAMERA position, not walking direction!
     * - If you walk right but phone points left, ARCore sees camera moving right
     * - But camera orientation is still left, creating coordinate mismatch
     * - This is why SLAM movement can seem slow or wrong compared to DR
     */
    private fun processArCoreFrame(frame: Frame): Pose2D? {
        try {
            val camera = frame.camera
            if (camera.trackingState != TrackingState.TRACKING) {
                return null
            }

            val pose = camera.displayOrientedPose
            
            // ARCore provides poses in meters
            // tx() = X translation in meters (left-right)
            // tz() = Z translation in meters (forward-backward, ARCore's forward is -Z)
            val x = pose.tx()
            val z = pose.tz()
            val yaw = MathUtils.quaternionToYaw(pose.qx(), pose.qy(), pose.qz(), pose.qw())
            
            // ARCore's forward axis is -Z, so negate Z to match our coordinate convention
            val currentPose = Pose2D(x, -z, yaw)

            if (initialPose == null) {
                initialPose = currentPose
                lastArCorePose = currentPose
                lastArCoreUpdateTime = System.currentTimeMillis()
                Log.d(TAG, "*** ARCore SLAM INITIALIZED at origin: ($x, ${-z}) ***")
                Log.d(TAG, "ARCore tracks CAMERA position in world space")
                Log.d(TAG, "NOTE: Camera movement may not match walking direction!")
                return Pose2D(0f, 0f, 0f)
            }

            // Calculate relative pose from initial position
            val relativePose = Pose2D(
                currentPose.x - initialPose!!.x,
                currentPose.y - initialPose!!.y,
                currentPose.theta - initialPose!!.theta
            )
            
            // Calculate velocity for debugging
            val currentTime = System.currentTimeMillis()
            if (lastArCorePose != null && lastArCoreUpdateTime > 0) {
                val dt = (currentTime - lastArCoreUpdateTime) / 1000f
                if (dt > 0.1f) { // Update velocity every 100ms
                    val dx = relativePose.x - (lastArCorePose!!.x - initialPose!!.x)
                    val dy = relativePose.y - (lastArCorePose!!.y - initialPose!!.y)
                    val speed = kotlin.math.sqrt(dx * dx + dy * dy) / dt
                    arCoreVelocity = Pose2D(dx / dt, dy / dt, 0f)
                    
                    if (frameCount % 60 == 0L) {
                        Log.d(TAG, "ARCore: pos=(${relativePose.x}, ${relativePose.y}), speed=${speed}m/s")
                        Log.d(TAG, "ARCore velocity: (${arCoreVelocity!!.x}, ${arCoreVelocity!!.y}) m/s")
                    }
                    
                    lastArCorePose = currentPose
                    lastArCoreUpdateTime = currentTime
                }
            } else {
                lastArCorePose = currentPose
                lastArCoreUpdateTime = currentTime
            }
            
            return relativePose
        } catch (e: Exception) {
            Log.e(TAG, "Error processing ARCore frame: ${e.message}")
            return null
        }
    }

    /**
     * Process simulated SLAM (fallback method - when ARCore not accessible).
     * 
     * IMPORTANT: This is SIMULATED movement, not real SLAM!
     * - Moves forward at fixed speed regardless of actual movement
     * - Does NOT track your real walking direction
     * - This is why you see straight line behavior
     * 
     * Real SLAM would use ARCore to track camera position, but that also
     * has limitations (tracks camera, not walking direction).
     */
    private fun processSimulatedSlam(currentTime: Long): Pose2D? {
        if (initialPose == null) {
            initialPose = Pose2D(0f, 0f, 0f)
            Log.d(TAG, "*** Using SIMULATED SLAM (ARCore not accessible) ***")
            Log.d(TAG, "WARNING: This is simulated movement, not real SLAM!")
            Log.d(TAG, "Simulated SLAM moves forward at fixed speed (1.2 m/s)")
            Log.d(TAG, "It does NOT track your actual walking direction")
        }

        // Update every ~100ms (10 Hz) for smooth movement
        val elapsedSeconds = (currentTime - lastUpdateTime) / 1000f
        if (elapsedSeconds < 0.1f) return lastKnownPose

        val lastPose = lastKnownPose ?: Pose2D(0f, 0f, 0f)
        
        // Simulate movement at realistic walking speed
        // Normal walking speed: ~1.2 m/s (4.3 km/h)
        // NOTE: This moves forward regardless of actual movement direction!
        val speed = 1.2f
        val distance = speed * elapsedSeconds
        
        // Move forward in current direction (but direction doesn't change based on walking)
        val newX = lastPose.x + distance * cos(lastPose.theta)
        val newY = lastPose.y + distance * sin(lastPose.theta)
        
        // Keep same heading (simulated SLAM doesn't respond to turns)
        val simulatedPose = Pose2D(newX, newY, lastPose.theta)
        
        // Log occasionally to verify it's working
        if (frameCount % 60 == 0L) {
            val totalDistance = kotlin.math.sqrt(newX * newX + newY * newY)
            Log.d(TAG, "Simulated SLAM: pos=($newX, $newY), total=$totalDistance m, speed=$speed m/s")
            Log.d(TAG, "NOTE: This is simulated - it doesn't track your real movement!")
        }
        
        return simulatedPose
    }

    /**
     * Get ARCore session using reflection.
     */
    private fun getArCoreSession(): Session? {
        val fieldNames = listOf("session", "arSession", "arCoreSession", "mSession", "_session")

        // Try direct access
        for (fieldName in fieldNames) {
            try {
                val field = arSceneView.javaClass.getDeclaredField(fieldName)
                field.isAccessible = true
                val value = field.get(arSceneView)
                if (value is Session) {
                    Log.d(TAG, "Found session via field: $fieldName")
                    return value
                }
            } catch (e: Exception) {
                // Continue
            }
        }

        // Try through arSessionView
        try {
            val arSessionViewField = arSceneView.javaClass.getDeclaredField("arSessionView")
            arSessionViewField.isAccessible = true
            val arSessionView = arSessionViewField.get(arSceneView)
            if (arSessionView != null) {
                for (fieldName in fieldNames) {
                    try {
                        val field = arSessionView.javaClass.getDeclaredField(fieldName)
                        field.isAccessible = true
                        val value = field.get(arSessionView)
                        if (value is Session) {
                            Log.d(TAG, "Found session via arSessionView.$fieldName")
                            return value
                        }
                    } catch (e: Exception) {
                        // Continue
                    }
                }
            }
        } catch (e: Exception) {
            // Ignore
        }

        return null
    }

    /**
     * Configure ARCore session for plane detection.
     */
    private fun configureSessionForPlaneDetection(session: Session) {
        try {
            val config = com.google.ar.core.Config(session)
            config.focusMode = com.google.ar.core.Config.FocusMode.AUTO
            config.planeFindingMode = com.google.ar.core.Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            config.updateMode = com.google.ar.core.Config.UpdateMode.LATEST_CAMERA_IMAGE
            session.configure(config)
            Log.d(TAG, "Configured ARCore for plane detection")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to configure session", e)
        }
    }

    /**
     * Stop tracking.
     */
    fun stop() {
        if (!isTracking) return

        Log.d(TAG, "=== STOPPING SLAM TRACKING ===")
        isTracking = false
        sessionPollJob?.cancel()
        sessionPollJob = null

        framePollRunnable?.let {
            handler.removeCallbacks(it)
            framePollRunnable = null
        }

        initialPose = null
        session = null
        lastKnownPose = null
        useFallback = false
        Log.d(TAG, "SLAM stopped. Processed $frameCount frames")
    }

    /**
     * Reset tracker.
     */
    fun reset() {
        initialPose = null
        lastKnownPose = null
        frameCount = 0
        listener.invoke(Pose2D(0f, 0f, 0f))
    }
}
