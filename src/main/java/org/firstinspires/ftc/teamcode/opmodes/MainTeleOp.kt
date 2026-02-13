package org.firstinspires.ftc.teamcode.opmodes

import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.*

/**
 * Main TeleOp with 3-button control and automatic shooting subsystems.
 * 
 * Controls:
 * - RT: Shoot (automatic aim + flywheel + hood)
 * - LT: Intake
 * - X: Outtake
 * - Field-centric mecanum drive (always active)
 * 
 * Automatic features:
 * - Turret always points at goal (Kalman filter fusion)
 * - Flywheel speed adjusts based on distance
 * - Hood angle adjusts based on distance
 * - Shooting zone detection via barycentric coordinates
 */
class MainTeleOp : NextFTCOpMode() {
    
    // Goal positions (TUNE for your field!)
    private val BLUE_GOAL_X = 72.0
    private val BLUE_GOAL_Y = -36.0  // Adjust for your field
    private val RED_GOAL_X = 72.0
    private val RED_GOAL_Y = 36.0    // Adjust for your field
    
    // Current goal (changes based on alliance)
    private var goalX = BLUE_GOAL_X
    private var goalY = BLUE_GOAL_Y
    private var isBlueAlliance = true
    
    // Limelight instance (TUNE the name!)
    // private var limelight: Limelight? = null
    
    init {
        addComponents(
            PedroComponent(org.firstinspires.ftc.teamcode.util.Constants::createFollower),
            SubsystemComponent(
                Drive,
                Intake,
                Gate,
                FlyWheel,
                Hood,
                Turret
            ),
            BulkReadComponent,
            BindingsComponent
        )
    }
    
    override fun onInit() {
        // Set goal based on alliance (TUNE THIS!)
        // For now, default to blue
        goalX = BLUE_GOAL_X
        goalY = BLUE_GOAL_Y
        Drive.setGoalPosition(goalX, goalY)
        
        // Initialize Pedro pose (TUNE THIS!)
        follower.pose = Pose(0.0, 0.0, 0.0)
    }
    
    override fun onStartButtonPressed() {
        // Start field-centric driver control
        // TUNE: Adjust stick mappings for your controller
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,  // Forward/back
            -Gamepads.gamepad1.leftStickX,  // Strafe
            -Gamepads.gamepad1.rightStickX,  // Rotate
            false  // Field-centric
        )()
        
        // Setup button bindings
        setupBindings()
    }
    
    private fun setupBindings() {
        // ============================================
        // SHOOT - RT (Automatic aim + fire)
        // ============================================
        Gamepads.gamepad1.rightTrigger greaterThan 0.5 thenApply {
            shoot()
        } otherwise {
            stopShooting()
        }
        
        // ============================================
        // INTAKE - LT
        // ============================================
        Gamepads.gamepad1.leftTrigger greaterThan 0.5 thenApply {
            Intake.intake()
        } otherwise {
            Intake.stop()
        }
        
        // ============================================
        // OUTTAKE - X button
        // ============================================
        Gamepads.gamepad1.x whenBecomesTrue {
            Intake.outtake()
        } whenBecomesFalse {
            Intake.stop()
        }
        
        // ============================================
        // Manual overrides (for testing)
        // ============================================
        
        // Y: Toggle gate manually
        Gamepads.gamepad1.y whenBecomesTrue {
            Gate.toggle()
        }
        
        // A: Stop everything
        Gamepads.gamepad1.a whenBecomesTrue {
            allStop()
        }
        
        // D-pad: Manual hood positions (testing)
        Gamepads.gamepad1.dpadUp whenBecomesTrue { Hood.open() }
        Gamepads.gamepad1.dpadDown whenBecomesTrue { Hood.close() }
        
        // Bumpers: Manual flywheel (testing)
        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            FlyWheel.setRPM(1300.0)  // Test RPM
        } whenBecomesFalse {
            FlyWheel.stop()
        }
        
        Gamepads.gamepad1.leftBumper whenBecomesTrue {
            FlyWheel.setRPM(1000.0)  // Test RPM
        } whenBecomesFalse {
            FlyWheel.stop()
        }
    }
    
    /**
     * Automatic shooting sequence
     */
    private fun shoot() {
        // 1. Get distance to goal
        val distance = Drive.distanceToGoal()
        
        // 2. Update all subsystems based on distance
        FlyWheel.setSpeedForDistance(distance)
        Hood.setForDistance(distance)
        
        // 3. Update turret with Kalman filter fusion
        // Note: Replace null with actual Limelight tx reading
        val llTx: Double? = null  // TODO: Get from Limelight
        
        Turret.updateAim(
            Drive.robotX,
            Drive.robotY,
            Drive.robotHeading,
            goalX,
            goalY,
            llTx
        )
        
        // 4. Open gate to shoot
        Gate.open()
    }
    
    /**
     * Stop shooting sequence
     */
    private fun stopShooting() {
        FlyWheel.stop()
        Gate.close()
        Turret.stop()
    }
    
    /**
     * Emergency stop everything
     */
    private fun allStop() {
        Intake.stop()
        FlyWheel.stop()
        Gate.close()
        Turret.stop()
    }
    
    override fun onUpdate() {
        // CRITICAL: Update drive pose every loop
        Drive.updatePose()
        
        // Check if in shooting zone
        val inZone = Drive.isInShootingZone()
        val baryCoords = Drive.getBarycentricCoords()
        
        // Telemetry
        telemetry.addData("=== DRIVE ===", "")
        telemetry.addData("X", "%.1f".format(Drive.robotX))
        telemetry.addData("Y", "%.1f".format(Drive.robotY))
        telemetry.addData("Heading", "%.1f°".format(Math.toDegrees(Drive.robotHeading)))
        telemetry.addData("Dist to Goal", "%.1f".format(Drive.distanceToGoal()))
        
        telemetry.addData("=== SHOOTING ===", "")
        telemetry.addData("In Zone", inZone)
        telemetry.addData("Bary U", "%.2f".format(baryCoords.first))
        telemetry.addData("Bary V", "%.2f".format(baryCoords.second))
        telemetry.addData("Bary W", "%.2f".format(baryCoords.third))
        
        telemetry.addData("=== SYSTEMS ===", "")
        telemetry.addData("Flywheel RPM", "%.0f".format(FlyWheel.velocity))
        telemetry.addData("Target RPM", "%.0f".format(FlyWheel.targetRPM))
        telemetry.addData("Hood Pos", "%.2f".format(Hood.position))
        telemetry.addData("Turret Aim", if (Turret.isAiming) "YES" else "NO")
        telemetry.addData("Odo Source", if (Turret.usingOdometry) "YES" else "NO")
        telemetry.addData("LL Source", if (Turret.usingLimelight) "YES" else "NO")
        
        telemetry.addData("=== INTAKE ===", "")
        telemetry.addData("Intaking", if (Intake.isIntaking) "YES" else "NO")
        telemetry.addData("Outtaking", if (Intake.isOuttaking) "YES" else "NO")
        telemetry.addData("Has Piece", if (Intake.hasGamePiece()) "YES" else "NO")
    }
}
