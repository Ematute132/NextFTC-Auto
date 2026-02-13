package org.firstinspires.ftc.teamcode.opmodes.testing

import com.pedropathing.geometry.Pose
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.extensions.pedro.PedroDriverControlled
import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent
import org.firstinspires.ftc.teamcode.subsystems.*
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlyWheel
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import org.firstinspires.ftc.teamcode.subsystems.shooter.Turret

/**
 * Full System Integration Test
 * Tests all subsystems together in teleop
 */
class FullSystemTest : NextFTCOpMode() {
    
    private val goalX = 72.0
    private val goalY = 0.0
    
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
        follower.pose = Pose(48.0, 0.0, 0.0)
    }
    
    override fun onStartButtonPressed() {
        // Field-centric drive
        PedroDriverControlled(
            -Gamepads.gamepad1.leftStickY,
            -Gamepads.gamepad1.leftStickX,
            -Gamepads.gamepad1.rightStickX,
            false
        )()
        
        // RT = Shoot (auto all)
        Gamepads.gamepad1.rightTrigger greaterThan 0.5 thenApply {
            shoot()
        } otherwise {
            stopShooting()
        }
        
        // LT = Intake
        Gamepads.gamepad1.leftTrigger greaterThan 0.5 thenApply {
            Intake.intake()
        } otherwise {
            Intake.stop()
        }
        
        // X = Outtake
        Gamepads.gamepad1.x whenBecomesTrue {
            Intake.outtake()
        } whenBecomesFalse {
            Intake.stop()
        }
        
        // Y = Toggle gate
        Gamepads.gamepad1.y whenBecomesTrue {
            Gate.toggle()
        }
        
        // A = All stop
        Gamepads.gamepad1.a whenBecomesTrue {
            allStop()
        }
        
        // Bumpers = Manual flywheel
        Gamepads.gamepad1.rightBumper whenBecomesTrue {
            FlyWheel.setRPM(1300.0)
        } whenBecomesFalse {
            FlyWheel.stop()
        }
        
        Gamepads.gamepad1.leftBumper whenBecomesTrue {
            FlyWheel.setRPM(1000.0)
        } whenBecomesFalse {
            FlyWheel.stop()
        }
    }
    
    private fun shoot() {
        val dist = Drive.distanceToGoal()
        FlyWheel.setSpeedForDistance(dist)
        Hood.setForDistance(dist)
        Turret.updateAim(Drive.robotX, Drive.robotY, Drive.robotHeading, goalX, goalY, null)
        Gate.open()
    }
    
    private fun stopShooting() {
        FlyWheel.stop()
        Gate.close()
        Turret.stop()
    }
    
    private fun allStop() {
        Intake.stop()
        FlyWheel.stop()
        Gate.close()
        Turret.stop()
    }
    
    override fun onUpdate() {
        Drive.updatePose()
        
        // DRIVE
        telemetry.addData("=== DRIVE ===", "")
        telemetry.addData("X", "%.1f".format(Drive.robotX))
        telemetry.addData("Y", "%.1f".format(Drive.robotY))
        telemetry.addData("H", "%.1f°".format(Math.toDegrees(Drive.robotHeading)))
        telemetry.addData("Dist", "%.1f".format(Drive.distanceToGoal()))
        
        // SHOOTING
        telemetry.addData("=== SHOOTING ===", "")
        telemetry.addData("In Zone", if (Drive.isInShootingZone()) "YES" else "NO")
        telemetry.addData("FlyRPM", "%.0f/%.0f".format(FlyWheel.velocity, FlyWheel.targetRPM))
        telemetry.addData("Hood", "%.2f".format(Hood.position))
        telemetry.addData("Turret", "%.1f°".format(Math.toDegrees(Turret.angle)))
        
        // INTAKE
        telemetry.addData("=== INTAKE ===", "")
        telemetry.addData("Intake", if (Intake.isIntaking) "ON" else "OFF")
        telemetry.addData("Out", if (Intake.isOuttaking) "ON" else "OFF")
        telemetry.addData("Has Piece", if (Intake.hasGamePiece()) "YES" else "NO")
        
        // GATE
        telemetry.addData("=== GATE ===", "")
        telemetry.addData("Open", if (Gate.isOpen) "YES" else "NO")
    }
}
