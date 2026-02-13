package org.firstinspires.ftc.teamcode.opmodes.testing

import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.Drive
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.components.BulkReadComponent
import com.pedropathing.geometry.Pose

/**
 * Test Drive subsystem
 * - Check odometry pose updates
 * - Test barycentric zone detection
 * - Verify distance/angle calculations
 */
class DriveTest : NextFTCOpMode() {
    
    init {
        addComponents(
            PedroComponent(org.firstinspires.ftc.teamcode.util.Constants::createFollower),
            SubsystemComponent(Drive),
            BulkReadComponent
        )
    }
    
    override fun onInit() {
        // Set starting pose
        follower.pose = Pose(48.0, 0.0, 0.0)  // Start in middle of field
    }
    
    override fun onStartButtonPressed() {
        // Use gamepad to simulate movement for testing
    }
    
    override fun onUpdate() {
        // Update pose
        Drive.updatePose()
        
        // Display drive data
        telemetry.addData("=== DRIVE TEST ===", "")
        telemetry.addData("Pose X", "%.1f".format(Drive.robotX))
        telemetry.addData("Pose Y", "%.1f".format(Drive.robotY))
        telemetry.addData("Heading", "%.1f°".format(Math.toDegrees(Drive.robotHeading)))
        
        telemetry.addData("=== SHOOTING ZONE ===", "")
        telemetry.addData("Distance to Goal", "%.1f".format(Drive.distanceToGoal()))
        telemetry.addData("Angle to Goal", "%.1f°".format(Math.toDegrees(Drive.angleToGoal())))
        telemetry.addData("In Shooting Zone", if (Drive.isInShootingZone()) "YES" else "NO")
        
        val bary = Drive.getBarycentricCoords()
        telemetry.addData("Bary U", "%.3f".format(bary.first))
        telemetry.addData("Bary V", "%.3f".format(bary.second))
        telemetry.addData("Bary W", "%.3f".format(bary.third))
        
        telemetry.addData("=== INSTRUCTIONS ===", "")
        telemetry.addData("Use D-pad to move robot", "")
        telemetry.addData("to test zone detection", "")
    }
}
