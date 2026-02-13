package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx

/**
 * Intake subsystem for game piece collection.
 * Simple 3-button control: intake, outtake, stop
 */
object Intake : Subsystem {
    
    private val motor = MotorEx("intake").reversed()
    
    // Power settings (TUNE THESE!)
    const val INTAKE_POWER = 1.0
    const val OUTTAKE_POWER = -0.8
    const val STOP_POWER = 0.0
    
    // Current state
    var isIntaking = false
    var isOuttaking = false
    
    /**
     * Start intaking (sucks in game pieces)
     */
    fun intake() {
        motor.power = INTAKE_POWER
        isIntaking = true
        isOuttaking = false
    }
    
    /**
     * Start outtaking (spits out game pieces)
     */
    fun outtake() {
        motor.power = OUTTAKE_POWER
        isIntaking = false
        isOuttaking = true
    }
    
    /**
     * Stop intake
     */
    fun stop() {
        motor.power = STOP_POWER
        isIntaking = false
        isOuttaking = false
    }
    
    /**
     * Toggle between intake and stop
     */
    fun toggle() {
        if (isIntaking) {
            stop()
        } else {
            intake()
        }
    }
    
    /**
     * Get motor current for game piece detection (TUNE THRESHOLD!)
     */
    val current: Double get() = motor.current
    
    /**
     * Check if we have a game piece
     */
    fun hasGamePiece(threshold: Double = 2.5): Boolean = current > threshold
    
    override fun periodic() {
        // State is updated in intake()/outtake()/stop()
    }
}
