package com.example.smartnav.ui

import androidx.lifecycle.LiveData
import androidx.lifecycle.MutableLiveData
import androidx.lifecycle.ViewModel
import com.example.smartnav.data.Pose2D

class SharedViewModel : ViewModel() {

    private val _drPoses = MutableLiveData<List<Pose2D>>(emptyList())
    val drPoses: LiveData<List<Pose2D>> = _drPoses

    private val _slamPoses = MutableLiveData<List<Pose2D>>(emptyList())
    val slamPoses: LiveData<List<Pose2D>> = _slamPoses

    private val _tracking = MutableLiveData(false)
    val tracking: LiveData<Boolean> = _tracking

    fun addDrPose(pose: Pose2D) {
        val currentPoses = _drPoses.value ?: emptyList()
        _drPoses.postValue(currentPoses + pose)
    }

    fun addSlamPose(pose: Pose2D) {
        val currentPoses = _slamPoses.value ?: emptyList()
        _slamPoses.postValue(currentPoses + pose)
    }

    fun startTracking() {
        _tracking.postValue(true)
    }

    fun stopTracking() {
        _tracking.postValue(false)
    }

    fun reset() {
        _drPoses.postValue(emptyList())
        _slamPoses.postValue(emptyList())
    }
}
