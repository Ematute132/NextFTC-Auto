package org.firstinspires.ftc.teamcode.subsystems

import dev.nextftc.core.subsystems.Subsystem
import dev.nextftc.extensions.pedro.PedroComponent.Companion.follower

/**
 * Drive subsystem using Pedro Pathing odometry.
 * Provides field-centric drive and shooting zone detection.
 */
object Drive : Subsystem {
    
    // Robot dimensions (TUNE THESE for your robot!)
    const val ROBOT_WIDTH = 18.0  // inches
    const val ROBOT_LENGTH = 18.0 // inches
    
    // Shooting zone corners (TUNE THESE for your field!)
    // These define a triangular shooting zone
    data class ZonePoint(val x: Double, val y: Double)
    
    // Default shooting zone (center field, adjustable)
    var goalX = 72.0   // Center of opposing goal
    var goalY = 0.0
    
    // Shooting zone as triangle (adjust for your field)
    private val shootingZone = arrayOf(
        ZonePoint(goalX, goalY - 24.0),  // Bottom of zone
        ZonePoint(goalX, goalY + 24.0),  // Top of zone  
        ZonePoint(goalX - 36.0, goalY)   // Back of zone (robot must be in front)
    )
    
    /**
     * Update the goal position (for different alliances)
     */
    fun setGoalPosition(x: Double, y: Double) {
        goalX = x
        goalY = y
        updateShootingZone()
    }
    
    /**
     * Update shooting zone based on current goal
     */
    private fun updateShootingZone() {
        shootingZone[0] = ZonePoint(goalX, goalY - 24.0)
        shootingZone[1] = ZonePoint(goalX, goalY + 24.0)
        shootingZone[2] = ZonePoint(goalX - 36.0, goalY)
    }
    
    /**
     * Get current robot position from Pedro odometry
     */
    val robotX: Double get() = follower.pose.x
    val robotY: Double get() = follower.pose.y
    val robotHeading: Double get() = follower.pose.heading
    
    /**
     * Calculate distance to goal using odometry
     */
    fun distanceToGoal(): Double {
        val dx = goalX - robotX
        val dy = goalY - robotY
        return kotlin.math.hypot(dx, dy)
    }
    
    /**
     * Calculate angle to goal from robot heading
     */
    fun angleToGoal(): Double {
        val dx = goalX - robotX
        val dy = goalY - robotY
        val fieldAngle = kotlin.math.atan2(dy, dx)
        return fieldAngle - robotHeading
    }
    
    /**
     * Check if robot is within shooting zone using barycentric coordinates
     * Returns true if robot can shoot from current position
     */
    fun isInShootingZone(): Boolean {
        val px = robotX
        val py = robotY
        
        val p0 = shootingZone[0]
        val p1 = shootingZone[1]
        val p2 = shootingZone[2]
        
        // Barycentric coordinate calculation
        val denom = (p1.y - p2.y) * (p0.x - p2.x) + (p2.x - p1.x) * (p0.y - p2.y)
        
        if (denom == 0.0) return false
        
        val u = ((p1.y - p2.y) * (px - p2.x) + (p2.x - p1.x) * (py - p2.y)) / denom
        val v = ((p2.y - p0.y) * (px - p2.x) + (p0.x - p2.x) * (py - p2.y)) / denom
        val w = 1.0 - u - v
        
        // Check if point is inside triangle
        return u >= 0 && v >= 0 && w >= 0
    }
    
    /**
     * Get barycentric coordinates for debugging
     */
    fun getBarycentricCoords(): Triple<Double, Double, Double> {
        val px = robotX
        val py = robotY
        
        val p0 = shootingZone[0]
        val p1 = shootingZone[1]
        val p2 = shootingZone[2]
        
        val denom = (p1.y - p2.y) * (p0.x - p2.x) + (p2.x - p1.x) * (p0.y - p2.y)
        
        if (denom == 0.0) return Triple(0.0, 0.0, 0.0)
        
        val u = ((p1.y - p2.y) * (px - p2.x) + (p2.x - p1.x) * (py - p2.y)) / denom
        val v = ((p2.y - p0.y) * (px - p2.x) + (p0.x - p2.x) * (py - p2.y)) / denom
        val w = 1.0 - u - v
        
        return Triple(u, v, w)
    }
    
    /**
     * CRITICAL: Call this in periodic() to update pose
     */
    fun updatePose() {
        follower.update()
    }
    
    /**
     * Reset robot pose (for autonomous start)
     */
    fun resetPose(x: Double, y: Double, heading: Double) {
        follower.pose = com.pedropathing.geometry.Pose(x, y, heading)
    }
    
    override fun periodic() {
        follower.update()
    }
}
