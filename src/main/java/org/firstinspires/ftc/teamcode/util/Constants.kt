package org.firstinspires.ftc.teamcode.util

import com.pedropathing.follower.Follower
import com.pedropathing.localization.Pose
import com.pedropathing.pathing.BezierLine
import com.pedropathing.pathing.PathBuilder
import com.pedropathing.pathing.PathRewriter
import com.pedropathing.util.Constants
import com.qualcomm.robotcore.hardware.HardwareMap

/**
 * Pedro Pathing Constants
 * TUNE THESE for your robot!
 */
object Constants {
    
    // ============================================
    // ODOMETRY WHEEL CONFIGURATION
    // ============================================
    // Encoder ticks per revolution
    const val ODO_TICKS_PER_REV = 537.7
    
    // Wheel diameter in inches
    const val ODO_WHEEL_DIAMETER = 2.0
    
    // Distance between left and right odometry wheels (inches)
    const val ODO_TRACK_WIDTH = 12.0
    
    // Distance from center to front odometry wheel (inches)
    const val ODO_FRONT_OFFSET = 4.0
    
    // ============================================
    // MOTOR CONFIGURATION
    // ============================================
    // Motor ticks per revolution
    const val MOTOR_TICKS_PER_REV = 537.7
    
    // Max velocity for velocity control
    const val MAX_VELOCITY = 2800.0
    
    // ============================================
    // DRIVE CONFIGURATION
    // ============================================
    // Mecanum wheel radius (inches)
    const val WHEEL_RADIUS = 2.0
    
    // Gear ratio from motor to wheel
    const val GEAR_RATIO = 1.0
    
    // Track width (distance between left and right wheels)
    const val TRACK_WIDTH = 16.0
    
    // ============================================
    // FOLOOWER CONFIGURATION
    // ============================================
    // Create the follower with odometry
    fun createFollower(hardwareMap: HardwareMap): Follower {
        val follower = Follower(hardwareMap)
        
        // Set odometry positions (TUNE for your robot!)
        follower.setPoseUpdate { 
            Pose(
                x = 0.0,  // TODO: Read from odometry encoders
                y = 0.0,
                heading = 0.0
            )
        }
        
        return follower
    }
}
