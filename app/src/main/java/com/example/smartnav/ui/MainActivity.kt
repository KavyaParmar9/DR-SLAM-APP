package com.example.smartnav.ui

import android.Manifest
import android.content.pm.PackageManager
import android.os.Bundle
import android.widget.Toast
import androidx.activity.result.contract.ActivityResultContracts
import androidx.appcompat.app.AppCompatActivity
import androidx.core.content.ContextCompat
import androidx.lifecycle.lifecycleScope
import com.example.smartnav.R
import com.example.smartnav.data.Pose2D
import com.example.smartnav.data.TrajectoryRepository
import com.example.smartnav.databinding.ActivityMainBinding
import com.example.smartnav.dr.DeadReckoningEngine
import com.example.smartnav.slam.OrbSlamTracker
import io.github.sceneview.ar.ARSceneView
import kotlinx.coroutines.Dispatchers
import kotlinx.coroutines.launch
import kotlinx.coroutines.withContext
import java.util.Locale
import kotlin.math.sqrt

class MainActivity : AppCompatActivity() {

    private lateinit var binding: ActivityMainBinding
    private lateinit var deadReckoningEngine: DeadReckoningEngine
    private lateinit var slamTracker: OrbSlamTracker
    private lateinit var slamSceneView: ARSceneView

    private val repository = TrajectoryRepository()

    private val cameraPermissionLauncher =
        registerForActivityResult(ActivityResultContracts.RequestPermission()) { granted ->
            if (granted) {
                startTrackingInternal()
            } else {
                Toast.makeText(this, "Camera permission is required for ARCore", Toast.LENGTH_LONG)
                    .show()
            }
        }

    override fun onCreate(savedInstanceState: Bundle?) {
        super.onCreate(savedInstanceState)
        binding = ActivityMainBinding.inflate(layoutInflater)
        setContentView(binding.root)

        slamSceneView = binding.slamSceneView // Use binding to get the view

        // Initialize ARSceneView - ensure it's ready
        initializeSlamSceneView()

        slamTracker = OrbSlamTracker(this, slamSceneView) { pose ->
            android.util.Log.d("MainActivity", "*** ORB-SLAM LISTENER CALLED: (${pose.x}, ${pose.y}) ***")
            android.util.Log.d("MainActivity", "Adding SLAM pose to repository: (${pose.x}, ${pose.y})")
            repository.addSlamPose(pose)
            android.util.Log.d("MainActivity", "Updating UI with SLAM pose: (${pose.x}, ${pose.y})")
            updateUi(poseType = PoseType.SLAM, pose = pose)
            android.util.Log.d("MainActivity", "UI updated, repository has ${repository.getSlam().size} SLAM poses")
        }

        deadReckoningEngine = DeadReckoningEngine(this) { pose ->
            repository.addDeadReckoningPose(pose)
            updateUi(poseType = PoseType.DR, pose = pose)
        }

        setupUi()
    }

    private fun initializeSlamSceneView() {
        // Ensure ARSceneView is visible and ready
        slamSceneView.visibility = android.view.View.VISIBLE
        
        // Configure ARCore session when available
        try {
            // Try to configure plane detection
            val scene = slamSceneView.scene
            // SceneView should handle ARCore initialization automatically
        } catch (e: Exception) {
            android.util.Log.e("MainActivity", "Error accessing scene", e)
        }
    }

    override fun onResume() {
        super.onResume()
        // ARSceneView lifecycle is managed automatically, but ensure it's visible
        slamSceneView.visibility = android.view.View.VISIBLE
    }

    override fun onPause() {
        super.onPause()
        stopTracking()
    }

    private fun setupUi() = with(binding) {
        startButton.setOnClickListener { startTracking() }
        stopButton.setOnClickListener { stopTracking() }
        resetButton.setOnClickListener { resetSession() }
        saveButton.setOnClickListener { saveSession() }
        loadButton.setOnClickListener { loadSession() }
        updatePoseLabels(null, null)
    }

    private fun startTracking() {
        if (ContextCompat.checkSelfPermission(this, Manifest.permission.CAMERA) ==
            PackageManager.PERMISSION_GRANTED
        ) {
            startTrackingInternal()
        } else {
            cameraPermissionLauncher.launch(Manifest.permission.CAMERA)
        }
    }

    private fun startTrackingInternal() {
        android.util.Log.d("MainActivity", "=== START TRACKING INTERNAL ===")
        deadReckoningEngine.start()
        slamTracker.start()
        Toast.makeText(this, "ORB-SLAM tracking started - detecting features and building map", Toast.LENGTH_LONG).show()
    }

    private fun stopTracking() {
        if (::deadReckoningEngine.isInitialized) {
            deadReckoningEngine.stop()
        }
        if (::slamTracker.isInitialized) {
            slamTracker.stop()
        }
    }

    private fun resetSession() {
        repository.reset()
        if (::deadReckoningEngine.isInitialized) {
            deadReckoningEngine.reset()
        }
        if (::slamTracker.isInitialized) {
            slamTracker.reset()
        }
        binding.trajectoryView.setTrajectories(
            repository.getDeadReckoning(),
            repository.getSlam()
        )
        updatePoseLabels(null, null)
    }

    private fun saveSession() {
        lifecycleScope.launch(Dispatchers.IO) {
            val file = repository.saveToCsv(this@MainActivity)
            val message = if (file != null) {
                "Saved session to ${file.absolutePath}"
            } else {
                "Nothing to save yet"
            }
            withContext(Dispatchers.Main) {
                Toast.makeText(this@MainActivity, message, Toast.LENGTH_LONG).show()
            }
        }
    }

    private fun loadSession() {
        lifecycleScope.launch(Dispatchers.IO) {
            val session = repository.loadLastSession(this@MainActivity)
            withContext(Dispatchers.Main) {
                if (session == null) {
                    Toast.makeText(this@MainActivity, "No saved sessions found", Toast.LENGTH_SHORT)
                        .show()
                    return@withContext
                }
                val (dr, slam) = session
                binding.trajectoryView.setTrajectories(dr, slam)
                updatePoseLabels(dr.lastOrNull(), slam.lastOrNull())
            }
        }
    }

    private fun updateUi(poseType: PoseType, pose: Pose2D) {
        runOnUiThread {
            when (poseType) {
                PoseType.DR -> binding.drPoseText.text =
                    getString(R.string.dr_pose_label) + ": ${pose.toDisplay()}"

                PoseType.SLAM -> binding.slamPoseText.text =
                    getString(R.string.slam_pose_label) + ": ${pose.toDisplay()}"
            }
            binding.trajectoryView.setTrajectories(
                repository.getDeadReckoning(),
                repository.getSlam()
            )
            binding.driftText.text =
                getString(R.string.drift_label) + ": ${formatMeters(repository.computeDrift())}"
        }
    }

    private fun updatePoseLabels(dr: Pose2D?, slam: Pose2D?) {
        binding.drPoseText.text =
            getString(R.string.dr_pose_label) + ": ${dr?.toDisplay() ?: "(0,0) m"}"
        binding.slamPoseText.text =
            getString(R.string.slam_pose_label) + ": ${slam?.toDisplay() ?: "(0,0) m"}"
        val drift = if (dr != null && slam != null) {
            val dx = slam.x - dr.x
            val dy = slam.y - dr.y
            sqrt(dx * dx + dy * dy)
        } else 0f
        binding.driftText.text =
            getString(R.string.drift_label) + ": ${formatMeters(drift)}"
    }

    private fun Pose2D.toDisplay(): String =
        String.format(Locale.US, "(%.2f, %.2f) m", x, y)

    private fun formatMeters(value: Float): String =
        String.format(Locale.US, "%.2f m", value)

    override fun onDestroy() {
        stopTracking()
        super.onDestroy()
    }

    private enum class PoseType { DR, SLAM }
}
