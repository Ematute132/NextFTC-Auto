package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

/**
 * Flywheel with NextFTC PID + Feedforward control.
 * Automatic speed adjustment based on distance.
 */
@Configurable
object FlyWheel : Subsystem {
    
    private val motor1 = MotorEx("Fly1")
    private val motor2 = MotorEx("Fly2").reversed()
    
    // PID + FF Coefficients (TUNE THESE!)
    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.003, 0.08, 0.0) // kV, kA, kS
    @JvmField var pidCoefficients = PIDCoefficients(0.009, 0.0, 0.01) // kP, kI, kD
    
    // NextFTC Control System
    private val controller: ControlSystem = controlSystem {
        basicFF(ffCoefficients)
        velPid(pidCoefficients)
    }
    
    // Distance to RPM mapping (TUNE THESE!)
    private val distanceToRPM = listOf(
        0.0 to 0.0,
        12.0 to 800.0,
        18.0 to 1000.0,
        24.0 to 1150.0,
        30.0 to 1250.0,
        36.0 to 1350.0,
        42.0 to 1450.0,
        48.0 to 1550.0,
        60.0 to 1650.0
    )
    
    // Current state
    var targetRPM = 0.0
    var currentDistance = 999.0
    var isRunning = false
    
    /**
     * Set flywheel speed automatically based on distance
     */
    fun setSpeedForDistance(distance: Double) {
        currentDistance = distance
        
        var rpm = 0.0
        for ((dist, r) in distanceToRPM) {
            if (distance <= dist) {
                rpm = r
                break
            }
        }
        
        setRPM(rpm)
    }
    
    /**
     * Set target RPM (uses NextFTC ControlSystem)
     */
    fun setRPM(rpm: Double) {
        targetRPM = rpm
        // KineticState(position, velocity) - position=0 for velocity-only control
        controller.goal = KineticState(0.0, rpm)
        isRunning = rpm > 50
    }
    
    /**
     * Manual power override
     */
    fun setPower(power: Double) {
        isRunning = false
        controller.goal = KineticState(0.0, 0.0)
        motor1.power = power.coerceIn(-1.0, 1.0)
        motor2.power = power.coerceIn(-1.0, 1.0)
    }
    
    /**
     * Stop flywheel
     */
    fun stop() {
        targetRPM = 0.0
        controller.goal = KineticState(0.0, 0.0)
        isRunning = false
    }
    
    /**
     * Get current velocity
     */
    val velocity: Double get() = motor1.velocity
    
    /**
     * Check if at target speed
     */
    fun isAtSpeed(tolerance: Double = 100.0): Boolean {
        return abs(velocity - targetRPM) < tolerance
    }
    
    /**
     * Check motor sync
     */
    fun isSynced(tolerance: Double = 150.0): Boolean {
        return abs(motor1.velocity - motor2.velocity) < tolerance
    }
    
    override fun periodic() {
        if (!isRunning) {
            motor1.power = 0.0
            motor2.power = 0.0
            return
        }
        
        // NextFTC PID + FF control
        // Uses motor1 as feedback source
        val controlEffort = controller.calculate(motor1.state)
        
        // Apply to both motors (clamped for safety)
        val clampedPower = controlEffort.coerceIn(-0.85, 0.85)
        motor1.power = clampedPower
        motor2.power = clampedPower
        
        // Telemetry
        PanelsTelemetry.telemetry.addData("Flywheel Power", "%.3f".format(clampedPower))
        PanelsTelemetry.telemetry.addData("Flywheel Target RPM", "%.0f".format(targetRPM))
        PanelsTelemetry.telemetry.addData("Flywheel Actual RPM", "%.0f".format(velocity))
        PanelsTelemetry.telemetry.addData("Flywheel Motor 2 RPM", "%.0f".format(motor2.velocity))
        PanelsTelemetry.telemetry.addData("Flywheel Synced", if (isSynced()) "YES" else "NO")
        PanelsTelemetry.telemetry.addData("Flywheel At Speed", if (isAtSpeed()) "YES" else "NO")
    }
}
