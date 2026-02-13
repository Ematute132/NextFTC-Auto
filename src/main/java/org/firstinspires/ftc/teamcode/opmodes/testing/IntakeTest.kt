package org.firstinspires.ftc.teamcode.opmodes.testing

import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.Intake
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.components.BulkReadComponent

/**
 * Test Intake subsystem
 * - RT: Intake
 * - LT: Outtake
 * - X: Stop
 * - Shows motor current for game piece detection
 */
class IntakeTest : NextFTCOpMode() {
    
    init {
        addComponents(
            SubsystemComponent(Intake),
            BulkReadComponent
        )
    }
    
    override fun onStartButtonPressed() {
        // Button bindings
        Gamepads.gamepad1.rightTrigger greaterThan 0.5 thenApply {
            Intake.intake()
        } otherwise {
            Intake.stop()
        }
        
        Gamepads.gamepad1.leftTrigger greaterThan 0.5 thenApply {
            Intake.outtake()
        }
        
        Gamepads.gamepad1.x whenBecomesTrue {
            Intake.stop()
        }
    }
    
    override fun onUpdate() {
        telemetry.addData("=== INTAKE TEST ===", "")
        telemetry.addData("RT = Intake", "")
        telemetry.addData("LT = Outtake", "")
        telemetry.addData("X = Stop", "")
        
        telemetry.addData("--- STATUS ---", "")
        telemetry.addData("Intaking", if (Intake.isIntaking) "YES" else "NO")
        telemetry.addData("Outtaking", if (Intake.isOuttaking) "YES" else "NO")
        telemetry.addData("Motor Current", "%.2f A".format(Intake.current))
        telemetry.addData("Has Game Piece", if (Intake.hasGamePiece()) "YES" else "NO")
        
        telemetry.addData("--- TUNING ---", "")
        telemetry.addData("INTAKE_POWER", Intake.INTAKE_POWER)
        telemetry.addData("OUTTAKE_POWER", Intake.OUTTAKE_POWER)
    }
}
