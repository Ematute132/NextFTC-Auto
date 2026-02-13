package org.firstinspires.ftc.teamcode.opmodes.testing

import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.Gate
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.components.BulkReadComponent

/**
 * Test Gate subsystem
 * - A: Open gate
 * - B: Close gate
 * - X: Toggle
 * - Tests servo control
 */
class GateTest : NextFTCOpMode() {
    
    init {
        addComponents(
            SubsystemComponent(Gate),
            BulkReadComponent
        )
    }
    
    override fun onStartButtonPressed() {
        // Open gate
        Gamepads.gamepad1.a whenBecomesTrue {
            Gate.open()
        }
        
        // Close gate
        Gamepads.gamepad1.b whenBecomesTrue {
            Gate.close()
        }
        
        // Toggle
        Gamepads.gamepad1.x whenBecomesTrue {
            Gate.toggle()
        }
    }
    
    override fun onUpdate() {
        telemetry.addData("=== GATE TEST ===", "")
        telemetry.addData("A = Open", "")
        telemetry.addData("B = Close", "")
        telemetry.addData("X = Toggle", "")
        
        telemetry.addData("--- STATUS ---", "")
        telemetry.addData("Open", if (Gate.isOpen) "YES" else "NO")
        
        telemetry.addData("--- TUNING ---", "")
        telemetry.addData("OPEN_POSITION", Gate.OPEN_POSITION)
        telemetry.addData("CLOSED_POSITION", Gate.CLOSED_POSITION)
    }
}
