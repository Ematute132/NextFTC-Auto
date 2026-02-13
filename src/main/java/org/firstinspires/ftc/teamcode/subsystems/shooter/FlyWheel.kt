package org.firstinspires.ftc.teamcode.subsystems.shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

/**
 * Flywheel subsystem with automatic speed adjustment based on distance.
 * Uses distance from Kalman filter + odometry fusion.
 */
object FlyWheel : Subsystem {
    
    private val motor1 = MotorEx("Fly1")
    private val motor2 = MotorEx("Fly2").reversed()
    
    // Velocity presets based on distance (TUNE THESE for your robot!)
    // Format: distance in inches -> RPM
    private val distanceToRPM = mapOf(
        0.0 to 0.0,      // Stop
        12.0 to 800.0,   // Very close
        18.0 to 1000.0,  // Close
        24.0 to 1150.0,  // Medium-close
        30.0 to 1250.0,  // Medium
        36.0 to 1350.0,  // Medium-far
        42.0 to 1450.0,  // Far
        48.0 to 1550.0,  // Very far
        60.0 to 1650.0,  // Max range
        999.0 to 1700.0  // Beyond max
    )
    
    // Current state
    var targetRPM = 0.0
    var currentDistance = 999.0
    var isRunning = false
    
    /**
     * Set flywheel speed based on distance (automatic)
     * Uses odometry + Kalman filter distance
     */
    fun setSpeedForDistance(distance: Double) {
        currentDistance = distance
        
        // Find the appropriate RPM for this distance
        var rpm = 0.0
        for ((dist, r) in distanceToRPM.entries.sortedBy { it.key }) {
            if (distance <= dist) {
                rpm = r
                break
            }
        }
        
        targetRPM = rpm
        isRunning = rpm > 100
    }
    
    /**
     * Manual RPM setting (override automatic)
     */
    fun setRPM(rpm: Double) {
        targetRPM = rpm
        isRunning = rpm > 100
    }
    
    /**
     * Get current average velocity
     */
    val velocity: Double get() = (motor1.velocity + motor2.velocity) / 2.0
    
    /**
     * Check if at target speed (within tolerance)
     */
    fun isAtSpeed(tolerance: Double = 100.0): Boolean {
        return kotlin.math.abs(velocity - targetRPM) < tolerance
    }
    
    /**
     * Stop flywheel
     */
    fun stop() {
        targetRPM = 0.0
        motor1.power = 0.0
        motor2.power = 0.0
        isRunning = false
    }
    
    /**
     * Get motor synchronization status
     */
    fun isSynced(tolerance: Double = 150.0): Boolean {
        return kotlin.math.abs(motor1.velocity - motor2.velocity) < tolerance
    }
    
    override fun periodic() {
        if (!isRunning) {
            motor1.power = 0.0
            motor2.power = 0.0
            return
        }
        
        // Simple velocity control (TUNE for your robot!)
        // In production, use PID + Feedforward
        val error = targetRPM - velocity
        val kP = 0.003  // Proportional gain
        
        var power = error * kP
        power = power.coerceIn(-0.85, 0.85)
        
        motor1.power = power
        motor2.power = power
    }
}
