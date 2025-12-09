package com.example.smartnav.ui

import android.content.Context
import android.graphics.Canvas
import android.graphics.Color
import android.graphics.Paint
import android.graphics.Path
import android.util.AttributeSet
import android.view.View
import androidx.core.content.ContextCompat
import com.example.smartnav.R
import com.example.smartnav.data.Pose2D
import kotlin.math.max
import kotlin.math.min

class TrajectoryView @JvmOverloads constructor(
    context: Context,
    attrs: AttributeSet? = null,
    defStyleAttr: Int = 0
) : View(context, attrs, defStyleAttr) {

    private val drPoints = mutableListOf<Pose2D>()
    private val slamPoints = mutableListOf<Pose2D>()

    private val gridPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = Color.DKGRAY
        strokeWidth = 1f
        style = Paint.Style.STROKE
    }

    private val drPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.dr_path)
        strokeWidth = 8f
        style = Paint.Style.STROKE
    }

    private val slamPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        color = ContextCompat.getColor(context, R.color.slam_path)
        strokeWidth = 8f
        style = Paint.Style.STROKE
    }

    private val legendPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
        textSize = 32f
    }

    fun setTrajectories(dr: List<Pose2D>, slam: List<Pose2D>) {
        drPoints.clear()
        drPoints.addAll(dr)
        slamPoints.clear()
        slamPoints.addAll(slam)
        invalidate()
    }

    override fun onDraw(canvas: Canvas) {
        super.onDraw(canvas)
        if (drPoints.isEmpty() && slamPoints.isEmpty()) {
            drawCenterText(canvas, "Paths will appear here")
            return
        }
        drawGrid(canvas)
        drawPath(canvas, drPoints, drPaint)
        drawPath(canvas, slamPoints, slamPaint)
        drawLegend(canvas)
    }

    private fun drawGrid(canvas: Canvas) {
        val step = width.toFloat() / 10f
        for (i in 0..10) {
            val x = i * step
            canvas.drawLine(x, 0f, x, height.toFloat(), gridPaint)
        }
        val yStep = height.toFloat() / 10f
        for (i in 0..10) {
            val y = i * yStep
            canvas.drawLine(0f, y, width.toFloat(), y, gridPaint)
        }
    }

    private fun drawPath(canvas: Canvas, points: List<Pose2D>, paint: Paint) {
        if (points.size < 2) return
        val bounds = computeBounds()
        val scale = computeScale(bounds)
        val offsetX = (width - bounds.width * scale) / 2f - bounds.minX * scale
        val offsetY = (height - bounds.height * scale) / 2f - bounds.minY * scale

        val path = Path()
        points.forEachIndexed { index, pose ->
            val x = pose.x * scale + offsetX
            val y = pose.y * scale + offsetY
            if (index == 0) path.moveTo(x, y) else path.lineTo(x, y)
        }
        canvas.drawPath(path, paint)
    }

    private fun drawLegend(canvas: Canvas) {
        legendPaint.color = drPaint.color
        canvas.drawRect(20f, 20f, 60f, 60f, legendPaint)
        canvas.drawText("Dead Reckoning", 80f, 55f, legendPaint)

        legendPaint.color = slamPaint.color
        canvas.drawRect(20f, 80f, 60f, 120f, legendPaint)
        canvas.drawText("SLAM", 80f, 115f, legendPaint)
    }

    private fun computeScale(bounds: Bounds): Float {
        val maxSpan = max(bounds.width, bounds.height).coerceAtLeast(1f)
        val drawableSize = min(width, height) * 0.9f
        return drawableSize / maxSpan
    }

    private fun computeBounds(): Bounds {
        var minX = Float.MAX_VALUE
        var maxX = Float.MIN_VALUE
        var minY = Float.MAX_VALUE
        var maxY = Float.MIN_VALUE
        (drPoints + slamPoints).forEach {
            minX = min(minX, it.x)
            maxX = max(maxX, it.x)
            minY = min(minY, it.y)
            maxY = max(maxY, it.y)
        }
        if (minX == Float.MAX_VALUE) return Bounds(0f, 0f, 0f, 0f)
        return Bounds(maxX - minX, maxY - minY, minX, minY)
    }

    private fun drawCenterText(canvas: Canvas, text: String) {
        val textPaint = Paint(Paint.ANTI_ALIAS_FLAG).apply {
            color = Color.WHITE
            textSize = 36f
            textAlign = Paint.Align.CENTER
        }
        canvas.drawText(text, width / 2f, height / 2f, textPaint)
    }

    private data class Bounds(
        val width: Float,
        val height: Float,
        val minX: Float,
        val minY: Float
    )
}
