package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.ServoEx

/**
 * Gate subsystem for controlling game piece release.
 */
object Gate : Subsystem {
    
    private val servo = ServoEx("gate")
    
    // Position settings (TUNE THESE!)
    const val OPEN_POSITION = 0.0
    const val CLOSED_POSITION = 1.0
    
    // Current state
    var isOpen = false
    
    /**
     * Open the gate (release game piece)
     */
    fun open() {
        servo.position = OPEN_POSITION
        isOpen = true
    }
    
    /**
     * Close the gate (hold game piece)
     */
    fun close() {
        servo.position = CLOSED_POSITION
        isOpen = false
    }
    
    /**
     * Toggle gate state
     */
    fun toggle() {
        if (isOpen) close() else open()
    }
    
    override fun periodic() {
        // Servo position is set directly
    }
}
