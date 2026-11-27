package com.example.smartnav.slam

import android.content.Context
import android.graphics.Bitmap
import android.graphics.BitmapFactory
import android.graphics.ImageFormat
import android.graphics.Rect
import android.graphics.YuvImage
import android.media.Image
import android.os.Handler
import android.os.Looper
import android.util.Log
import com.example.smartnav.data.Pose2D
import com.example.smartnav.utils.MathUtils
import com.google.ar.core.Frame
import com.google.ar.core.Session
import com.google.ar.core.TrackingState
import com.google.ar.core.exceptions.NotYetAvailableException
import io.github.sceneview.ar.ARSceneView
import kotlin.math.*
import java.nio.ByteBuffer


class OrbSlamTracker(
    private val context: Context,
    private val arSceneView: ARSceneView,
    private val listener: (Pose2D) -> Unit
) {
    
    private var isTracking = false
    private var initialPose: Pose2D? = null
    private var session: Session? = null
    private val handler = Handler(Looper.getMainLooper())
    private var framePollRunnable: Runnable? = null
    
    // Feature tracking
    private var previousFeatures: List<FeaturePoint>? = null
    private var previousFrameGray: ByteArray? = null
    private var previousFrameWidth = 0
    private var previousFrameHeight = 0
    
    // Map of tracked features
    private val mapPoints = mutableListOf<MapPoint>()
    
    // Current pose
    private var currentPose = Pose2D(0f, 0f, 0f)
    private var frameCount = 0L
    private var lastPoseUpdateTime = 0L  // Track when last pose was sent
    
    companion object {
        private const val TAG = "OrbSlamTracker"
        private const val MAX_FEATURES = 200
        private const val FEATURE_THRESHOLD = 30  // Lowered threshold for better detection
        private const val MATCH_DISTANCE_THRESHOLD = 30.0  // Increased for more matches
        private const val MIN_MATCHES = 3  // Lowered minimum matches
    }
    
    /**
     * Feature point with position and descriptor.
     */
    data class FeaturePoint(
        val x: Float,
        val y: Float,
        val response: Float,  // Corner strength
        val descriptor: ByteArray? = null  // Simplified descriptor (intensity patch)
    )
    
    /**
     * Map point in world coordinates.
     */
    data class MapPoint(
        val x: Float,
        val y: Float,
        val z: Float = 0f,
        val observations: Int = 1  // Number of times observed
    )
    
    /**
     * Start ORB-SLAM tracking.
     */
    fun start() {
        if (isTracking) return
        
        Log.d(TAG, "=== STARTING ORB-SLAM TRACKING ===")
        isTracking = true
        frameCount = 0
        initialPose = null
        currentPose = Pose2D(0f, 0f, 0f)
        mapPoints.clear()
        previousFeatures = null
        previousFrameGray = null
        
        // Try to get ARCore session
        session = getArCoreSession()
        if (session != null) {
            Log.d(TAG, "ARCore session found")
            configureSession(session!!)
        }
        
        // Start frame polling
        startFramePolling()
    }
    
    /**
     * Start polling for ARCore frames.
     */
    private fun startFramePolling() {
        framePollRunnable = object : Runnable {
            override fun run() {
                if (!isTracking) return
                
                try {
                    val currentSession = session ?: getArCoreSession()
                    if (currentSession != null) {
                        session = currentSession
                        try {
                            val frame = currentSession.update()
                            if (frame != null) {
                                processFrame(frame)
                            } else {
                                if (frameCount % 60 == 0L) {
                                    Log.d(TAG, "No frame available yet...")
                                }
                            }
                        } catch (e: Exception) {
                            // Session might be busy
                            if (frameCount % 60 == 0L) {
                                Log.d(TAG, "Session update failed: ${e.message}")
                            }
                        }
                    } else {
                        if (frameCount % 60 == 0L) {
                            Log.d(TAG, "ARCore session not available yet...")
                        }
                    }
                } catch (e: Exception) {
                    if (frameCount % 60 == 0L) {
                        Log.e(TAG, "Error in frame polling: ${e.message}")
                    }
                }
                
                // Poll at ~30 FPS (every 33ms) for smoother tracking
                handler.postDelayed(this, 33)
            }
        }
        
        handler.post(framePollRunnable!!)
        Log.d(TAG, "ORB-SLAM frame polling started")
    }
    
    /**
     * Process ARCore frame for ORB-SLAM.
     */
    private fun processFrame(frame: Frame) {
        try {
            val camera = frame.camera
            
            // Always try to get pose, even if not fully tracking
            // This ensures we send updates as soon as ARCore starts working
            if (camera.trackingState == TrackingState.TRACKING) {
                // ARCore is tracking - process frame normally
            } else {
                // Not tracking yet, but still try to send (0,0) to maintain connection
                if (frameCount % 30 == 0L) {
                    Log.d(TAG, "ARCore tracking state: ${camera.trackingState}")
                }
                // Still try to use pose if available (sometimes pose is available even if not TRACKING)
                try {
                    val pose = camera.displayOrientedPose
                    val x = pose.tx()
                    val z = pose.tz()
                    if (initialPose == null && (abs(x) > 0.001f || abs(z) > 0.001f)) {
                        // ARCore has pose data, initialize
                        val yaw = MathUtils.quaternionToYaw(pose.qx(), pose.qy(), pose.qz(), pose.qw())
                        val arCorePose = Pose2D(x, -z, yaw)
                        initialPose = arCorePose
                        currentPose = arCorePose
                        Log.d(TAG, "Initialized from non-tracking pose: ($x, ${-z})")
                        listener.invoke(Pose2D(0f, 0f, 0f))
                        lastPoseUpdateTime = System.currentTimeMillis()
                    }
                } catch (e: Exception) {
                    // Pose not available yet
                }
                return
            }
            
            // Get camera image
            val image = try {
                frame.acquireCameraImage()
            } catch (e: NotYetAvailableException) {
                if (frameCount % 30 == 0L) {
                    Log.d(TAG, "Camera image not ready yet: ${e.message}")
                }
                return
            } catch (e: IllegalStateException) {
                if (frameCount % 30 == 0L) {
                    Log.e(TAG, "Unable to access camera image: ${e.message}")
                }
                return
            }
            
            val imageWidth: Int
            val imageHeight: Int
            val grayImageBytes: ByteArray?

            try {
                imageWidth = image.width
                imageHeight = image.height
                grayImageBytes = imageToGrayscale(image)
            } finally {
                image.close()
            }

            if (grayImageBytes == null || grayImageBytes.isEmpty()) {
                if (frameCount % 30 == 0L) {
                    Log.w(TAG, "Failed to convert image to grayscale")
                }
                return
            }
            
            // Always use ARCore pose as primary method (since ARCore is working - white dots visible)
            // Feature detection can be used for enhancement, but ARCore provides reliable tracking
            useArCorePoseFallback(frame)
            
            // Also try feature detection for potential enhancement
            val features = detectFeatures(grayImageBytes, imageWidth, imageHeight)

            if (features.isEmpty()) {
                if (frameCount % 30 == 0L) {
                    Log.d(TAG, "No features detected, using ARCore pose (frame ${imageWidth}x${imageHeight})")
                }
                // ARCore fallback already called above, just return
                return
            }
            
            Log.d(TAG, "Detected ${features.size} features in frame ${imageWidth}x${imageHeight}")
            
            // If first frame, initialize map
            if (previousFeatures == null) {
                Log.d(TAG, "Initializing ORB-SLAM with first frame")
                initializeMap(features, grayImageBytes, imageWidth, imageHeight, frame)
                previousFeatures = features
                previousFrameGray = grayImageBytes
                previousFrameWidth = imageWidth
                previousFrameHeight = imageHeight
                // Send initial pose
                listener.invoke(Pose2D(0f, 0f, 0f))
                lastPoseUpdateTime = System.currentTimeMillis()
                return
            }
            
            // Track features from previous frame
            val matches = trackFeatures(
                previousFeatures!!,
                features,
                previousFrameGray!!,
                grayImageBytes,
                previousFrameWidth,
                previousFrameHeight,
                imageWidth,
                imageHeight
            )
            
            Log.d(TAG, "Matched ${matches.size} features")
            
            if (matches.size < MIN_MATCHES) {
                if (frameCount % 30 == 0L) {
                    Log.w(TAG, "Not enough matches: ${matches.size} (need ${MIN_MATCHES}), using ARCore fallback")
                }
                // Update previous frame for next attempt
                previousFeatures = features
                previousFrameGray = grayImageBytes
                previousFrameWidth = imageWidth
                previousFrameHeight = imageHeight
                // Use ARCore fallback when not enough matches
                useArCorePoseFallback(frame)
                return
            }
            
            // Estimate camera motion from feature matches
            val poseUpdate = estimateMotion(matches, frame)
            
            if (poseUpdate != null) {
                // Update current pose
                currentPose = Pose2D(
                    currentPose.x + poseUpdate.x,
                    currentPose.y + poseUpdate.y,
                    currentPose.theta + poseUpdate.theta
                )
                
                // Set initial pose if not set
                if (initialPose == null) {
                    initialPose = currentPose
                    Log.d(TAG, "ORB-SLAM initialized at origin")
                }
                
                // Calculate relative pose
                val relativePose = Pose2D(
                    currentPose.x - initialPose!!.x,
                    currentPose.y - initialPose!!.y,
                    currentPose.theta - initialPose!!.theta
                )
                
                // Update map with new observations
                updateMap(matches, features)
                
                // Notify listener
                listener.invoke(relativePose)
                lastPoseUpdateTime = System.currentTimeMillis()
                
                if (frameCount % 30 == 0L) {
                    Log.d(TAG, "ORB-SLAM pose: (${relativePose.x}, ${relativePose.y}), map points: ${mapPoints.size}")
                }
            } else {
                // If pose estimation failed, use ARCore fallback
                if (frameCount % 30 == 0L) {
                    Log.w(TAG, "Pose estimation failed, using ARCore fallback")
                }
                useArCorePoseFallback(frame)
            }
            
            // Update previous frame data
            previousFeatures = features
            previousFrameGray = grayImageBytes
            previousFrameWidth = imageWidth
            previousFrameHeight = imageHeight
            
            frameCount++
            
        } catch (e: Exception) {
            Log.e(TAG, "Error processing frame: ${e.message}", e)
        }
    }
    
    /**
     * Detect features (corners) in grayscale image using simplified corner detection.
     */
    private fun detectFeatures(grayImage: ByteArray, width: Int, height: Int): List<FeaturePoint> {
        val features = mutableListOf<FeaturePoint>()
        
        if (grayImage.size < width * height) {
            Log.w(TAG, "Image size mismatch: ${grayImage.size} < ${width * height}")
            return emptyList()
        }
        
        // Simplified corner detection using Harris corner response
        // We'll use a simple gradient-based corner detector
        val threshold = FEATURE_THRESHOLD
        val windowSize = 3
        
        // Use larger step to speed up detection
        val stepSize = 4
        
        for (y in windowSize until height - windowSize step stepSize) {
            for (x in windowSize until width - windowSize step stepSize) {
                val idx = y * width + x
                if (idx >= grayImage.size) continue
                
                // Calculate corner response (simplified)
                val response = calculateCornerResponse(grayImage, x, y, width, height, windowSize)
                
                if (response > threshold) {
                    // Extract descriptor (intensity patch around feature)
                    val descriptor = extractDescriptor(grayImage, x, y, width, height)
                    features.add(FeaturePoint(x.toFloat(), y.toFloat(), response, descriptor))
                    
                    if (features.size >= MAX_FEATURES) {
                        break
                    }
                }
            }
            if (features.size >= MAX_FEATURES) {
                break
            }
        }
        
        // Sort by response and take top features
        val sortedFeatures = features.sortedByDescending { it.response }.take(MAX_FEATURES)
        
        if (sortedFeatures.isNotEmpty()) {
            Log.d(TAG, "Feature detection: found ${sortedFeatures.size} features (max response: ${sortedFeatures.first().response})")
        }
        
        return sortedFeatures
    }
    
    /**
     * Calculate corner response using simplified Harris corner detector.
     */
    private fun calculateCornerResponse(
        image: ByteArray,
        x: Int,
        y: Int,
        width: Int,
        height: Int,
        windowSize: Int
    ): Float {
        var Ixx = 0.0
        var Iyy = 0.0
        var Ixy = 0.0
        
        val centerIdx = y * width + x
        if (centerIdx >= image.size) return 0f
        
        val centerValue = image[centerIdx].toInt() and 0xFF
        
        for (dy in -windowSize..windowSize) {
            for (dx in -windowSize..windowSize) {
                val px = x + dx
                val py = y + dy
                
                if (px < 0 || px >= width || py < 0 || py >= height) continue
                
                val idx = py * width + px
                if (idx >= image.size) continue
                
                // Calculate gradients (simplified)
                val gx = if (px + 1 < width && idx + 1 < image.size) {
                    val rightValue = image[idx + 1].toInt() and 0xFF
                    val currValue = image[idx].toInt() and 0xFF
                    rightValue - currValue
                } else 0
                
                val gy = if (py + 1 < height && idx + width < image.size) {
                    val bottomValue = image[idx + width].toInt() and 0xFF
                    val currValue = image[idx].toInt() and 0xFF
                    bottomValue - currValue
                } else 0
                
                Ixx += gx * gx
                Iyy += gy * gy
                Ixy += gx * gy
            }
        }
        
        // Harris corner response: det(M) - k*trace(M)^2
        val det = Ixx * Iyy - Ixy * Ixy
        val trace = Ixx + Iyy
        
        if (trace == 0.0) return 0f
        
        val k = 0.04
        val response = det - k * trace * trace
        
        // Normalize response to positive values
        return max(0f, response.toFloat())
    }
    
    /**
     * Extract descriptor (intensity patch) around feature point.
     */
    private fun extractDescriptor(
        image: ByteArray,
        x: Int,
        y: Int,
        width: Int,
        height: Int,
        patchSize: Int = 8
    ): ByteArray {
        val descriptor = ByteArray(patchSize * patchSize)
        var idx = 0
        
        for (dy in -patchSize/2 until patchSize/2) {
            for (dx in -patchSize/2 until patchSize/2) {
                val px = (x + dx).coerceIn(0, width - 1)
                val py = (y + dy).coerceIn(0, height - 1)
                val imgIdx = py * width + px
                descriptor[idx++] = image.getOrNull(imgIdx) ?: 0
            }
        }
        
        return descriptor
    }
    
    /**
     * Track features between frames using descriptor matching.
     */
    private fun trackFeatures(
        prevFeatures: List<FeaturePoint>,
        currFeatures: List<FeaturePoint>,
        prevImage: ByteArray,
        currImage: ByteArray,
        prevWidth: Int,
        prevHeight: Int,
        currWidth: Int,
        currHeight: Int
    ): List<FeatureMatch> {
        val matches = mutableListOf<FeatureMatch>()
        
        // Match features using descriptor distance
        for (prevFeat in prevFeatures) {
            var bestMatch: FeaturePoint? = null
            var bestDistance = Double.MAX_VALUE
            
            for (currFeat in currFeatures) {
                // Calculate descriptor distance
                val distance = calculateDescriptorDistance(prevFeat.descriptor, currFeat.descriptor)
                
                if (distance < bestDistance && distance < MATCH_DISTANCE_THRESHOLD) {
                    bestDistance = distance
                    bestMatch = currFeat
                }
            }
            
            if (bestMatch != null) {
                matches.add(FeatureMatch(prevFeat, bestMatch))
            }
        }
        
        return matches
    }
    
    /**
     * Calculate distance between descriptors (Hamming distance for binary descriptors).
     */
    private fun calculateDescriptorDistance(desc1: ByteArray?, desc2: ByteArray?): Double {
        if (desc1 == null || desc2 == null) return Double.MAX_VALUE
        if (desc1.size != desc2.size) return Double.MAX_VALUE
        
        var distance = 0.0
        for (i in desc1.indices) {
            val diff = (desc1[i].toInt() and 0xFF) - (desc2[i].toInt() and 0xFF)
            distance += diff * diff
        }
        
        return sqrt(distance)
    }
    
    /**
     * Feature match between two frames.
     */
    data class FeatureMatch(
        val prevFeature: FeaturePoint,
        val currFeature: FeaturePoint
    )
    
    /**
     * Estimate camera motion from feature matches.
     */
    private fun estimateMotion(matches: List<FeatureMatch>, frame: Frame): Pose2D? {
        if (matches.size < MIN_MATCHES) return null
        
        try {
            // Calculate average displacement
            var avgDx = 0.0
            var avgDy = 0.0
            
            for (match in matches) {
                val dx = match.currFeature.x - match.prevFeature.x
                val dy = match.currFeature.y - match.prevFeature.y
                avgDx += dx
                avgDy += dy
            }
            
            avgDx /= matches.size
            avgDy /= matches.size
            
            // Convert pixel displacement to meters
            // Use ARCore camera pose for scale if available
            val camera = frame.camera
            val pose = camera.displayOrientedPose
            
            // Get camera intrinsics for scale conversion
            // Simplified: assume ~1mm per pixel at typical phone distance
            val pixelToMeter = 0.001f
            
            // Scale displacement
            val dx = avgDx * pixelToMeter
            val dy = avgDy * pixelToMeter
            
            // Estimate rotation from feature movement
            var avgRotation = 0.0
            var rotationCount = 0
            
            for (match in matches.take(20)) {
                val dx = match.currFeature.x - match.prevFeature.x
                val dy = match.currFeature.y - match.prevFeature.y
                if (abs(dx) > 0.1 || abs(dy) > 0.1) {
                    val angle = atan2(dy.toDouble(), dx.toDouble())
                    avgRotation += angle
                    rotationCount++
                }
            }
            
            if (rotationCount > 0) {
                avgRotation /= rotationCount
            }
            
            // Use ARCore pose for better scale estimation
            val arCoreX = pose.tx()
            val arCoreZ = pose.tz()
            val arCoreYaw = MathUtils.quaternionToYaw(pose.qx(), pose.qy(), pose.qz(), pose.qw())
            
            // Combine feature-based motion with ARCore pose (weighted average)
            val featureWeight = 0.3f  // Weight for feature-based motion
            val arCoreWeight = 0.7f   // Weight for ARCore pose
            
            val combinedDx = dx * featureWeight + arCoreX * arCoreWeight
            val combinedDy = dy * featureWeight + (-arCoreZ) * arCoreWeight
            val combinedYaw = avgRotation.toFloat() * featureWeight + arCoreYaw * arCoreWeight
            
            return Pose2D(combinedDx.toFloat(), combinedDy.toFloat(), combinedYaw)
            
        } catch (e: Exception) {
            Log.e(TAG, "Error estimating motion: ${e.message}", e)
            return null
        }
    }
    
    /**
     * Initialize map with first frame features.
     */
    private fun initializeMap(
        features: List<FeaturePoint>,
        grayImage: ByteArray,
        width: Int,
        height: Int,
        frame: Frame
    ) {
        val camera = frame.camera
        val pose = camera.displayOrientedPose
        
        // Convert features to map points
        for (feature in features) {
            // Convert pixel coordinates to world coordinates (simplified)
            val worldX = feature.x * 0.001f  // Approximate conversion
            val worldY = feature.y * 0.001f
            val worldZ = 0f  // Assume planar motion
            
            mapPoints.add(MapPoint(worldX, worldY, worldZ, 1))
        }
        
        Log.d(TAG, "Initialized map with ${mapPoints.size} points")
    }
    
    /**
     * Update map with new feature observations.
     */
    private fun updateMap(matches: List<FeatureMatch>, newFeatures: List<FeaturePoint>) {
        // Update existing map points
        val matchedIndices = matches.map { it.currFeature }.toSet()
        
        // Add new unmatched features to map
        var newPoints = 0
        for (feature in newFeatures) {
            if (feature !in matchedIndices && mapPoints.size < 500) {
                val worldX = feature.x * 0.001f
                val worldY = feature.y * 0.001f
                mapPoints.add(MapPoint(worldX, worldY, 0f, 1))
                newPoints++
            }
        }
        
        if (newPoints > 0 && frameCount % 30 == 0L) {
            Log.d(TAG, "Added $newPoints new points to map (total: ${mapPoints.size})")
        }
    }
    
    /**
     * Convert Android Image to grayscale byte array.
     */
    private fun imageToGrayscale(image: Image): ByteArray? {
        return try {
            val yBuffer = image.planes[0].buffer
            val ySize = yBuffer.remaining()
            val yBytes = ByteArray(ySize)
            yBuffer.get(yBytes)
            yBytes
        } catch (e: Exception) {
            Log.e(TAG, "Error converting image to grayscale: ${e.message}", e)
            null
        }
    }
    
    /**
     * Primary method: Use ARCore pose directly.
     * Since ARCore is working (white dots visible), use it as primary tracking method.
     * This ensures SLAM always provides pose updates.
     */
    private fun useArCorePoseFallback(frame: Frame) {
        try {
            val camera = frame.camera
            if (camera.trackingState != TrackingState.TRACKING) {
                // If not tracking yet, send (0,0) to maintain connection
                if (initialPose == null) {
                    listener.invoke(Pose2D(0f, 0f, 0f))
                    lastPoseUpdateTime = System.currentTimeMillis()
                    if (frameCount % 30 == 0L) {
                        Log.d(TAG, "ARCore not tracking yet, sending (0,0)")
                    }
                } else {
                    // Send last known pose if available
                    val relativePose = Pose2D(
                        currentPose.x - initialPose!!.x,
                        currentPose.y - initialPose!!.y,
                        currentPose.theta - initialPose!!.theta
                    )
                    listener.invoke(relativePose)
                    lastPoseUpdateTime = System.currentTimeMillis()
                }
                return
            }
            
            val pose = camera.displayOrientedPose
            val x = pose.tx()
            val z = pose.tz()
            val yaw = MathUtils.quaternionToYaw(pose.qx(), pose.qy(), pose.qz(), pose.qw())
            
            val arCorePose = Pose2D(x, -z, yaw)
            
            if (initialPose == null) {
                initialPose = arCorePose
                currentPose = arCorePose
                Log.d(TAG, "*** ORB-SLAM initialized using ARCore at ($x, ${-z}) ***")
                listener.invoke(Pose2D(0f, 0f, 0f))
                lastPoseUpdateTime = System.currentTimeMillis()
                return
            }
            
            // Update current pose
            currentPose = arCorePose
            
            // Calculate relative pose from initial position
            val relativePose = Pose2D(
                currentPose.x - initialPose!!.x,
                currentPose.y - initialPose!!.y,
                currentPose.theta - initialPose!!.theta
            )
            
            // Always notify listener with updated pose
            listener.invoke(relativePose)
            lastPoseUpdateTime = System.currentTimeMillis()
            
            // Log every frame for debugging (remove % 30 to see all updates)
            if (frameCount % 30 == 0L) {
                Log.d(TAG, "ARCore SLAM pose: (${relativePose.x}, ${relativePose.y}), frame: $frameCount")
            }
        } catch (e: Exception) {
            Log.e(TAG, "Error in ARCore pose: ${e.message}", e)
            // Even on error, try to send pose to keep UI updated
            if (initialPose != null) {
                val relativePose = Pose2D(
                    currentPose.x - initialPose!!.x,
                    currentPose.y - initialPose!!.y,
                    currentPose.theta - initialPose!!.theta
                )
                listener.invoke(relativePose)
                lastPoseUpdateTime = System.currentTimeMillis()
            } else {
                listener.invoke(Pose2D(0f, 0f, 0f))
                lastPoseUpdateTime = System.currentTimeMillis()
            }
        }
    }
    
    /**
     * Get ARCore session using reflection.
     */
    private fun getArCoreSession(): Session? {
        val fieldNames = listOf("session", "arSession", "arCoreSession", "mSession", "_session")
        
        for (fieldName in fieldNames) {
            try {
                val field = arSceneView.javaClass.getDeclaredField(fieldName)
                field.isAccessible = true
                val value = field.get(arSceneView)
                if (value is Session) {
                    Log.d(TAG, "Found ARCore session via field: $fieldName")
                    return value
                }
            } catch (e: Exception) {
                // Continue
            }
        }
        
        return null
    }
    
    /**
     * Configure ARCore session.
     */
    private fun configureSession(session: Session) {
        try {
            val config = com.google.ar.core.Config(session)
            config.focusMode = com.google.ar.core.Config.FocusMode.AUTO
            config.planeFindingMode = com.google.ar.core.Config.PlaneFindingMode.HORIZONTAL_AND_VERTICAL
            session.configure(config)
            Log.d(TAG, "Configured ARCore session")
        } catch (e: Exception) {
            Log.e(TAG, "Failed to configure session: ${e.message}", e)
        }
    }
    
    /**
     * Stop tracking.
     */
    fun stop() {
        if (!isTracking) return
        
        Log.d(TAG, "=== STOPPING ORB-SLAM TRACKING ===")
        isTracking = false
        
        framePollRunnable?.let {
            handler.removeCallbacks(it)
            framePollRunnable = null
        }
        
        mapPoints.clear()
        previousFeatures = null
        previousFrameGray = null
        
        Log.d(TAG, "ORB-SLAM stopped. Processed $frameCount frames, final map has ${mapPoints.size} points")
    }
    
    /**
     * Reset tracker.
     */
    fun reset() {
        initialPose = null
        currentPose = Pose2D(0f, 0f, 0f)
        frameCount = 0
        mapPoints.clear()
        previousFeatures = null
        previousFrameGray = null
        listener.invoke(Pose2D(0f, 0f, 0f))
    }
    
    /**
     * Get current map points count.
     */
    fun getMapPointsCount(): Int = mapPoints.size
}
