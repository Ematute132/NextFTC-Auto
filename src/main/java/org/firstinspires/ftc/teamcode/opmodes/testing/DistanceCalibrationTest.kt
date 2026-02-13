package org.firstinspires.ftc.teamcode.opmodes.testing

import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.Drive
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret

/**
 * Distance Calibration Test
 * Use this to tune the flywheel RPM and hood position for different distances
 * 
 * Controls:
 * - D-pad: Move robot to specific distances from goal
 * - Shows actual distance, current RPM, current hood
 * - Record what RPM/hood works best at each distance
 */
class DistanceCalibrationTest : NextFTCOpMode() {
    
    private val goalX = 72.0
    private val goalY = 0.0
    
    // Test distances (inches from goal)
    private val testDistances = listOf(12.0, 18.0, 24.0, 30.0, 36.0, 42.0, 48.0)
    
    init {
        addComponents(
            PedroComponent(org.firstinspires.ftc.teamcode.util.Constants::createFollower),
            SubsystemComponent(Drive, FlyWheel, Hood, Turret),
            BulkReadComponent,
            BindingsComponent
        )
    }
    
    override fun onInit() {
        follower.pose = Pose(goalX - 24.0, goalY, 0.0) // Start 24" from goal
    }
    
    override fun onStartButtonPressed() {
        // Move to different distances based on D-pad
        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            follower.pose = Pose(goalX - 12.0, goalY, 0.0) // 12"
        }
        
        Gamepads.gamepad1.dpadRight whenBecomesTrue {
            follower.pose = Pose(goalX - 18.0, goalY, 0.0) // 18"
        }
        
        Gamepads.gamepad1.dpadDown whenBecomesTrue {
            follower.pose = Pose(goalX - 24.0, goalY, 0.0) // 24"
        }
        
        Gamepads.gamepad1.dpadLeft whenBecomesTrue {
            follower.pose = Pose(goalX - 30.0, goalY, 0.0) // 30"
        }
        
        // A: Start flywheel at current distance
        Gamepads.gamepad1.a whenBecomesTrue {
            val dist = Drive.distanceToGoal()
            FlyWheel.setSpeedForDistance(dist)
            Hood.setForDistance(dist)
        }
        
        // B: Stop flywheel
        Gamepads.gamepad1.b whenBecomesTrue {
            FlyWheel.stop()
        }
        
        // X: Show current settings
        Gamepads.gamepad1.x whenBecomesTrue {
            // Just for display
        }
    }
    
    override fun onUpdate() {
        Drive.updatePose()
        
        val distance = Drive.distanceToGoal()
        
        // Show distance recommendations
        telemetry.addData("=== DISTANCE CALIBRATION ===", "")
        telemetry.addData("Current Dist", "%.1f\"".format(distance))
        telemetry.addData("Current RPM", "%.0f".format(FlyWheel.targetRPM))
        telemetry.addData("Current Hood", "%.2f".format(Hood.position))
        
        telemetry.addData("=== RECOMMENDED ===", "")
        for (dist in testDistances) {
            val recommendedRPM = getRecommendedRPM(dist)
            val recommendedHood = getRecommendedHood(dist)
            telemetry.addData("%.0f\" ->".format(dist), "RPM: %.0f, Hood: %.2f".format(recommendedRPM, recommendedHood))
        }
        
        telemetry.addData("=== CONTROLS ===", "")
        telemetry.addData("D-pad", "Move to distance")
        telemetry.addData("A", "Auto set for current dist")
        telemetry.addData("B", "Stop flywheel")
        
        telemetry.addData("=== RECORD THESE ===", "")
        telemetry.addData("Find best RPM/Hood", "at each distance")
    }
    
    // These should match your FlyWheel/Hood distance maps
    private fun getRecommendedRPM(distance: Double): Double {
        return when {
            distance <= 12.0 -> 800.0
            distance <= 18.0 -> 1000.0
            distance <= 24.0 -> 1150.0
            distance <= 30.0 -> 1250.0
            distance <= 36.0 -> 1350.0
            distance <= 42.0 -> 1450.0
            distance <= 48.0 -> 1550.0
            else -> 1650.0
        }
    }
    
    private fun getRecommendedHood(distance: Double): Double {
        return when {
            distance <= 12.0 -> 0.1
            distance <= 18.0 -> 0.2
            distance <= 24.0 -> 0.3
            distance <= 30.0 -> 0.45
            distance <= 36.0 -> 0.55
            distance <= 42.0 -> 0.65
            distance <= 48.0 -> 0.75
            else -> 0.85
        }
    }
}
