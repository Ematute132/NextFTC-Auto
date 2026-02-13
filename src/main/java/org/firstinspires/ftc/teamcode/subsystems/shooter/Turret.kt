package org.firstinspires.ftc.teamcode.subsystems.shooter

import com.bylazar.configurables.annotations.Configurable
import com.bylazar.telemetry.PanelsTelemetry
import dev.nextftc.control.ControlSystem
import dev.nextftc.control.KineticState
import dev.nextftc.control.builder.controlSystem
import dev.nextftc.control.feedback.FeedbackType
import dev.nextftc.control.feedback.PIDCoefficients
import dev.nextftc.control.feedforward.BasicFeedforwardParameters
import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.hardware.impl.MotorEx
import kotlin.math.*

/**
 * Turret with NextFTC PID + Feedforward control.
 * Uses Kalman filter fusion for sensor fusion (odometry + Limelight).
 */
@Configurable
object Turret : Subsystem {
    
    private val motor = MotorEx("turret")
    
    // PID + FF Coefficients (TUNE THESE!)
    @JvmField var pidCoefficients = PIDCoefficients(0.002, 0.0, 0.0) // kP, kI, kD
    @JvmField var ffCoefficients = BasicFeedforwardParameters(0.25, 0.0, 0.0) // kV, kA, kS
    
    // NextFTC Control System for position control
    private val controller: ControlSystem = controlSystem {
        posPid(pidCoefficients) // Position PID
        basicFF(ffCoefficients) // Feedforward
    }
    
    // Kalman Filter State
    private var kfAngle = 0.0
    private var kfCovariance = 1.0
    
    // Noise parameters (TUNE THESE!)
    private val processNoise = 0.01
    private val odoNoise = 0.1
    private val llNoise = 0.15
    
    // Turret parameters (TUNE THESE!)
    private const val TICKS_PER_REV = 537.7
    private const val GEAR_RATIO = 3.62
    private const val RAD_PER_TICK = 2.0 * PI / (TICKS_PER_REV * GEAR_RATIO)
    
    // Control limits
    @JvmField var minPower = 0.1
    @JvmField var maxPower = 0.75
    
    // Current state
    var isAiming = false
    var currentHeadingToGoal = 0.0
    var usingOdometry = false
    var usingLimelight = false
    private var lastMeasurementSource = "none"
    
    /**
     * Kalman filter predict step
     */
    private fun predict() {
        kfAngle += 0.0
        kfCovariance += processNoise
    }
    
    /**
     * Kalman filter update from odometry measurement
     */
    fun updateFromOdometry(angleRadians: Double) {
        val innovation = angleRadians - kfAngle
        val S = kfCovariance + odoNoise
        val K = kfCovariance / S
        
        kfAngle += K * innovation
        kfCovariance = (1.0 - K) * kfCovariance
        
        usingOdometry = true
        lastMeasurementSource = "odometry"
    }
    
    /**
     * Kalman filter update from Limelight measurement
     */
    fun updateFromLimelight(angleRadians: Double) {
        val innovation = angleRadians - kfAngle
        val S = kfCovariance + llNoise
        val K = kfCovariance / S
        
        kfAngle += K * innovation
        kfCovariance = (1.0 - K) * kfCovariance
        
        usingLimelight = true
        lastMeasurementSource = "limelight"
    }
    
    /**
     * Calculate angle to goal using odometry
     */
    fun calculateOdoAngle(
        robotX: Double, 
        robotY: Double, 
        robotHeading: Double, 
        goalX: Double, 
        goalY: Double
    ): Double {
        val dx = goalX - robotX
        val dy = goalY - robotY
        val fieldAngle = atan2(dy, dx)
        return normalizeAngle(fieldAngle - robotHeading)
    }
    
    /**
     * Calculate angle from Limelight tx (degrees)
     */
    fun calculateLLAngle(txDegrees: Double): Double? {
        return if (txDegrees.isNaN() || txDegrees.isInfinite()) null
        else Math.toRadians(txDegrees)
    }
    
    /**
     * Normalize angle to [-PI, PI]
     */
    private fun normalizeAngle(angle: Double): Double {
        var a = angle % (2.0 * PI)
        if (a <= -PI) a += 2.0 * PI
        if (a > PI) a -= 2.0 * PI
        return a
    }
    
    /**
     * Main aiming update - call every loop
     * Fuses odometry + Limelight using Kalman filter
     */
    fun updateAim(
        robotX: Double,
        robotY: Double,
        robotHeading: Double,
        goalX: Double,
        goalY: Double,
        limelightTx: Double? // Degrees from Limelight, null if no target
    ) {
        isAiming = true
        usingOdometry = false
        usingLimelight = false
        
        // Predict step
        predict()
        
        // Update from odometry (always available)
        val odoAngle = calculateOdoAngle(robotX, robotY, robotHeading, goalX, goalY)
        updateFromOdometry(odoAngle)
        
        // Update from Limelight if target visible
        limelightTx?.let { tx ->
            calculateLLAngle(tx)?.let { llAngle ->
                updateFromLimelight(llAngle)
            }
        }
        
        // Use fused estimate
        currentHeadingToGoal = kfAngle
        
        // Set controller goal (position = encoder ticks, velocity = 0)
        val encoderTarget = kfAngle / RAD_PER_TICK
        controller.goal = KineticState(encoderTarget, 0.0)
    }
    
    /**
     * Stop aiming
     */
    fun stop() {
        isAiming = false
        controller.goal = KineticState(0.0, 0.0)
        motor.power = 0.0
        kfAngle = 0.0
        kfCovariance = 1.0
    }
    
    /**
     * Get current turret angle in radians
     */
    val angle: Double get() = motor.currentPosition * RAD_PER_TICK
    
    /**
     * Manual power control
     */
    fun setManualPower(power: Double) {
        isAiming = false
        motor.power = power.coerceIn(-maxPower, maxPower)
    }
    
    override fun periodic() {
        if (!isAiming) return
        
        // Calculate control effort using NextFTC PID
        val controlEffort = controller.calculate(motor.state)
        
        // Apply minimum power threshold for friction
        val error = controller.goal.position - motor.currentPosition
        val adjustedPower = if (abs(error) > 10) {
            controlEffort + (if (controlEffort >= 0) minPower else -minPower)
        } else {
            controlEffort
        }
        
        motor.power = adjustedPower.coerceIn(-maxPower, maxPower)
        
        // Telemetry
        PanelsTelemetry.telemetry.addData("Turret Aiming", if (isAiming) "YES" else "NO")
        PanelsTelemetry.telemetry.addData("Turret Angle (deg)", "%.1f".format(Math.toDegrees(angle)))
        PanelsTelemetry.telemetry.addData("Turret Goal (deg)", "%.1f".format(Math.toDegrees(currentHeadingToGoal)))
        PanelsTelemetry.telemetry.addData("Turret Power", "%.3f".format(motor.power))
        PanelsTelemetry.telemetry.addData("Turret Source", lastMeasurementSource)
        PanelsTelemetry.telemetry.addData("Turret KF Covariance", "%.4f".format(kfCovariance))
    }
}
