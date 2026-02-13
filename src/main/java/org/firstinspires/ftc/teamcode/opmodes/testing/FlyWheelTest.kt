package org.firstinspires.ftc.teamcode.opmodes.testing

import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.shooter.FlyWheel
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.components.BulkReadComponent
import dev.nextftc.core.components.BindingsComponent

/**
 * Test FlyWheel subsystem
 * - D-pad Up: Set RPM to 1000
 * - D-pad Right: Set RPM to 1250
 * - D-pad Down: Set RPM to 1500
 * - X: Stop
 * - Tests PID + Feedforward control
 */
class FlyWheelTest : NextFTCOpMode() {
    
    init {
        addComponents(
            SubsystemComponent(FlyWheel),
            BulkReadComponent,
            BindingsComponent
        )
    }
    
    override fun onStartButtonPressed() {
        // Manual RPM presets
        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            FlyWheel.setRPM(1000.0)
        }
        
        Gamepads.gamepad1.dpadRight whenBecomesTrue {
            FlyWheel.setRPM(1250.0)
        }
        
        Gamepads.gamepad1.dpadDown whenBecomesTrue {
            FlyWheel.setRPM(1500.0)
        }
        
        Gamepads.gamepad1.dpadLeft whenBecomesTrue {
            FlyWheel.setRPM(800.0)
        }
        
        // Stop
        Gamepads.gamepad1.x whenBecomesTrue {
            FlyWheel.stop()
        }
        
        // Manual power test
        Gamepads.gamepad1.a whenBecomesTrue {
            FlyWheel.setPower(0.5)
        }
        
        Gamepads.gamepad1.b whenBecomesTrue {
            FlyWheel.setPower(0.0)
        }
    }
    
    override fun onUpdate() {
        telemetry.addData("=== FLYWHEEL TEST ===", "")
        telemetry.addData("D-pad UP", "1000 RPM")
        telemetry.addData("D-pad RIGHT", "1250 RPM")
        telemetry.addData("D-pad DOWN", "1500 RPM")
        telemetry.addData("D-pad LEFT", "800 RPM")
        telemetry.addData("X = Stop", "")
        telemetry.addData("A = 50% Power", "")
        telemetry.addData("B = Power Off", "")
        
        telemetry.addData("--- STATUS ---", "")
        telemetry.addData("Target RPM", "%.0f".format(FlyWheel.targetRPM))
        telemetry.addData("Actual RPM", "%.0f".format(FlyWheel.velocity))
        telemetry.addData("At Speed", if (FlyWheel.isAtSpeed()) "YES" else "NO")
        telemetry.addData("Running", if (FlyWheel.isRunning) "YES" else "NO")
        
        telemetry.addData("--- MOTORS ---", "")
        telemetry.addData("Synced", if (FlyWheel.isSynced()) "YES" else "NO")
    }
}
