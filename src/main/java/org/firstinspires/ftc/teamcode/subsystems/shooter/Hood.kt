package org.firstinspires.ftc.teamcode.subsystems.shooter

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

/**
 * Hood subsystem with automatic angle adjustment based on distance.
 * Works with FlyWheel to optimize shot trajectory.
 */
object Hood : Subsystem {
    
    private val servo = ServoEx("hood")
    
    // Position presets based on distance (TUNE THESE for your robot!)
    // Format: distance in inches -> servo position
    private val distanceToPosition = mapOf(
        0.0 to 0.0,      // Stop/retract
        12.0 to 0.1,     // Very close - hood up (more arc)
        18.0 to 0.2,     // Close
        24.0 to 0.3,     // Medium-close
        30.0 to 0.45,    // Medium
        36.0 to 0.55,    // Medium-far
        42.0 to 0.65,    // Far
        48.0 to 0.75,    // Very far
        60.0 to 0.85,    // Max range
        999.0 to 0.9     // Beyond max
    )
    
    // Current state
    var targetPosition = 0.0
    var currentDistance = 999.0
    
    /**
     * Set hood position based on distance (automatic)
     */
    fun setForDistance(distance: Double) {
        currentDistance = distance
        
        // Find appropriate position for this distance
        var position = 0.0
        for ((dist, pos) in distanceToPosition.entries.sortedBy { it.key }) {
            if (distance <= dist) {
                position = pos
                break
            }
        }
        
        targetPosition = position
        servo.position = position
    }
    
    /**
     * Manual position setting (override automatic)
     */
    fun setPosition(position: Double) {
        targetPosition = position.coerceIn(0.0, 1.0)
        servo.position = targetPosition
    }
    
    /**
     * Get current position
     */
    val position: Double get() = servo.position
    
    /**
     * Preset positions
     */
    fun close() = setPosition(0.0)    // Tucked in
    fun open() = setPosition(1.0)      // Extended
    
    override fun periodic() {
        // Servo position is set directly, no periodic update needed
    }
}
