package com.example.smartnav.ui

import android.os.Bundle
import android.view.LayoutInflater
import android.view.View
import android.view.ViewGroup
import androidx.fragment.app.Fragment
import androidx.fragment.app.activityViewModels
import com.example.smartnav.R
import com.example.smartnav.data.Pose2D
import com.example.smartnav.databinding.FragmentHomeBinding
import java.util.Locale
import kotlin.math.sqrt

class HomeFragment : Fragment() {

    private var _binding: FragmentHomeBinding? = null
    private val binding get() = _binding!!

    private val sharedViewModel: SharedViewModel by activityViewModels()

    override fun onCreateView(
        inflater: LayoutInflater,
        container: ViewGroup?,
        savedInstanceState: Bundle?
    ): View {
        _binding = FragmentHomeBinding.inflate(inflater, container, false)
        return binding.root
    }

    override fun onViewCreated(view: View, savedInstanceState: Bundle?) {
        super.onViewCreated(view, savedInstanceState)

        setupUi()

        binding.slamPoseText.visibility = View.GONE
        binding.driftText.visibility = View.GONE

        sharedViewModel.drPoses.observe(viewLifecycleOwner) { poses ->
            updatePoseLabels(poses.lastOrNull(), sharedViewModel.slamPoses.value?.lastOrNull())
        }

        sharedViewModel.slamPoses.observe(viewLifecycleOwner) { poses ->
            updatePoseLabels(sharedViewModel.drPoses.value?.lastOrNull(), poses.lastOrNull())
        }
    }

    private fun setupUi() = with(binding) {
        startButton.setOnClickListener { sharedViewModel.startTracking() }
        stopButton.setOnClickListener { sharedViewModel.stopTracking() }
        resetButton.setOnClickListener { sharedViewModel.reset() }
        // TODO: Re-implement save and load functionality
        saveButton.visibility = View.GONE
        loadButton.visibility = View.GONE
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

    override fun onDestroyView() {
        super.onDestroyView()
        _binding = null
    }
}
