package org.firstinspires.ftc.teamcode.opmodes.testing

import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.Drive
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.components.BulkReadComponent
import com.pedropathing.geometry.Pose

/**
 * Test Turret subsystem
 * - A: Start aiming at goal (72, 0)
 * - B: Stop aiming
 * - X: Manual left
 * - Y: Manual right
 * - Tests Kalman filter + PID control
 */
class TurretTest : NextFTCOpMode() {
    
    // Test goal position
    private val goalX = 72.0
    private val goalY = 0.0
    
    init {
        addComponents(
            PedroComponent(org.firstinspires.ftc.teamcode.util.Constants::createFollower),
            SubsystemComponent(Drive, Turret),
            BulkReadComponent
        )
    }
    
    override fun onInit() {
        // Start in shooting position
        follower.pose = Pose(48.0, 0.0, 0.0)
    }
    
    override fun onStartButtonPressed() {
        // Start auto aiming
        Gamepads.gamepad1.a whenBecomesTrue {
            // A: Start aiming
            Turret.updateAim(
                Drive.robotX,
                Drive.robotY,
                Drive.robotHeading,
                goalX,
                goalY,
                null  // No Limelight
            )
        }
        
        // Stop aiming
        Gamepads.gamepad1.b whenBecomesTrue {
            Turret.stop()
        }
        
        // Manual control
        Gamepads.gamepad1.x whenBecomesTrue {
            Turret.setManualPower(-0.3)  // Left
        } whenBecomesFalse {
            Turret.setManualPower(0.0)
        }
        
        Gamepads.gamepad1.y whenBecomesTrue {
            Turret.setManualPower(0.3)  // Right
        } whenBecomesFalse {
            Turret.setManualPower(0.0)
        }
        
        // Test with different positions
        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            follower.pose = Pose(48.0, 0.0, 0.0)  // Front of zone
        }
        
        Gamepads.gamepad1.dpadDown whenBecomesTrue {
            follower.pose = Pose(24.0, 0.0, 0.0)  // Closer
        }
        
        Gamepads.gamepad1.dpadLeft whenBecomesTrue {
            follower.pose = Pose(48.0, -24.0, 0.0)  // Left side
        }
        
        Gamepads.gamepad1.dpadRight whenBecomesTrue {
            follower.pose = Pose(48.0, 24.0, 0.0)  // Right side
        }
    }
    
    override fun onUpdate() {
        // Update drive pose
        Drive.updatePose()
        
        telemetry.addData("=== TURRET TEST ===", "")
        telemetry.addData("A = Start Aiming", "")
        telemetry.addData("B = Stop", "")
        telemetry.addData("X = Manual Left", "")
        telemetry.addData("Y = Manual Right", "")
        
        telemetry.addData("--- POSE CONTROL ---", "")
        telemetry.addData("D-pad UP", "Move to (48, 0)")
        telemetry.addData("D-pad DOWN", "Move to (24, 0)")
        telemetry.addData("D-pad LEFT", "Move to (48, -24)")
        telemetry.addData("D-pad RIGHT", "Move to (48, 24)")
        
        telemetry.addData("--- STATUS ---", "")
        telemetry.addData("Aiming", if (Turret.isAiming) "YES" else "NO")
        telemetry.addData("Angle (deg)", "%.1f°".format(Math.toDegrees(Turret.angle)))
        telemetry.addData("Goal Angle (deg)", "%.1f°".format(Math.toDegrees(Turret.currentHeadingToGoal)))
        telemetry.addData("Motor Power", "%.3f".format(Turret.angle))
        
        telemetry.addData("--- SOURCES ---", "")
        telemetry.addData("Using Odo", if (Turret.usingOdometry) "YES" else "NO")
        telemetry.addData("Using LL", if (Turret.usingLimelight) "YES" else "NO")
        
        telemetry.addData("--- KALMAN ---", "")
        telemetry.addData("KF Angle (deg)", "%.1f°".format(Math.toDegrees(Turret.currentHeadingToGoal)))
        
        telemetry.addData("--- ROBOT ---", "")
        telemetry.addData("X", "%.1f".format(Drive.robotX))
        telemetry.addData("Y", "%.1f".format(Drive.robotY))
        telemetry.addData("Heading", "%.1f°".format(Math.toDegrees(Drive.robotHeading)))
    }
}
