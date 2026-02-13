package org.firstinspires.ftc.teamcode.subsystems.shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

/**
 * Turret subsystem with automatic aiming using Kalman filter fusion.
 * Fuses odometry + Limelight for accurate goal tracking.
 */
object Turret : Subsystem {
    
    private val motor = MotorEx("turret")
    
    // Kalman filter state
    private var kfAngle = 0.0
    private var kfCovariance = 1.0
    
    // Noise parameters (TUNE THESE!)
    private val processNoise = 0.01      // Odometry trust
    private val measurementNoiseOdo = 0.1 // Odometry measurement noise
    private val measurementNoiseLL = 0.15  // Limelight noise (higher = trust less)
    
    // Turret parameters (TUNE THESE!)
    private const val MOTOR_TICKS_PER_REV = 537.7
    private const val GEAR_RATIO = 3.62
    private const val RAD_PER_TICK = 2.0 * PI / (MOTOR_TICKS_PER_REV * GEAR_RATIO)
    
    // Current state
    var isAiming = false
    var currentHeadingToGoal = 0.0
    
    // Source tracking
    var usingOdometry = false
    var usingLimelight = false
    
    /**
     * Kalman filter predict step
     */
    private fun predict() {
        kfAngle += 0.0  // No process model, just propagate
        kfCovariance += processNoise
    }
    
    /**
     * Kalman filter update from odometry
     */
    fun updateFromOdometry(angleRadians: Double) {
        val measurement = angleRadians
        update(measurement, measurementNoiseOdo)
        usingOdometry = true
    }
    
    /**
     * Kalman filter update from Limelight
     */
    fun updateFromLimelight(angleRadians: Double) {
        val measurement = angleRadians
        update(measurement, measurementNoiseLL)
        usingLimelight = true
    }
    
    /**
     * Kalman filter update
     */
    private fun update(measurement: Double, noise: Double) {
        val innovation = measurement - kfAngle
        val S = kfCovariance + noise
        val K = kfCovariance / S
        
        kfAngle += K * innovation
        kfCovariance = (1.0 - K) * kfCovariance
    }
    
    /**
     * Calculate angle to goal using odometry
     */
    fun calculateOdoAngle(robotX: Double, robotY: Double, robotHeading: Double, goalX: Double, goalY: Double): Double {
        val dx = goalX - robotX
        val dy = goalY - robotY
        val fieldAngle = atan2(dy, dx)
        return fieldAngle - robotHeading
    }
    
    /**
     * Calculate angle to goal using Limelight (if target visible)
     * Returns null if no target
     */
    fun calculateLLAngle(tx: Double): Double? {
        // tx is degrees off-center from Limelight
        return if (tx.isNaN() || tx.isInfinite()) null
        else Math.toRadians(tx)
    }
    
    /**
     * Main aiming update - call this every loop
     * Fuses odometry + Limelight for best accuracy
     */
    fun updateAim(
        robotX: Double, 
        robotY: Double, 
        robotHeading: Double, 
        goalX: Double, 
        goalY: Double,
        limelightTx: Double?
    ) {
        isAiming = true
        usingOdometry = false
        usingLimelight = false
        
        // Predict step
        predict()
        
        // Update from odometry (always available)
        val odoAngle = calculateOdoAngle(robotX, robotY, robotHeading, goalX, goalY)
        updateFromOdometry(odoAngle)
        
        // Update from Limelight if available (better accuracy)
        val llAngle = limelightTx?.let { calculateLLAngle(it) }
        if (llAngle != null) {
            updateFromLimelight(llAngle)
        }
        
        // Use fused estimate
        currentHeadingToGoal = kfAngle
        
        // Apply to motor (TUNE GAIN!)
        val motorPower = kfAngle * 0.5  // Proportional control
        motor.power = motorPower.coerceIn(-0.5, 0.5)
    }
    
    /**
     * Stop aiming
     */
    fun stop() {
        isAiming = false
        motor.power = 0.0
        kfAngle = 0.0
        kfCovariance = 1.0
    }
    
    /**
     * Get current turret angle in radians
     */
    val angle: Double get() = motor.currentPosition * RAD_PER_TICK
    
    override fun periodic() {
        // Aiming is updated via updateAim() in main loop
    }
}
