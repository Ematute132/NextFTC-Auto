package org.firstinspires.ftc.teamcode.opmodes.testing

import dev.nextftc.ftc.Gamepads
import dev.nextftc.ftc.NextFTCOpMode
import org.firstinspires.ftc.teamcode.subsystems.shooter.Hood
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.ftc.components.BulkReadComponent

/**
 * Test Hood subsystem
 * - D-pad Up: Open (far)
 * - D-pad Down: Close (near)
 * - D-pad Left: Mid
 * - Tests servo position control
 */
class HoodTest : NextFTCOpMode() {
    
    init {
        addComponents(
            SubsystemComponent(Hood),
            BulkReadComponent
        )
    }
    
    override fun onStartButtonPressed() {
        // Manual position control
        Gamepads.gamepad1.dpadUp whenBecomesTrue {
            Hood.open()
        }
        
        Gamepads.gamepad1.dpadDown whenBecomesTrue {
            Hood.close()
        }
        
        Gamepads.gamepad1.dpadLeft whenBecomesTrue {
            Hood.setPosition(0.5)  // Mid
        }
        
        Gamepads.gamepad1.dpadRight whenBecomesTrue {
            Hood.setPosition(0.25)  // Quarter
        }
        
        // Test auto distance mode
        Gamepads.gamepad1.a whenBecomesTrue {
            Hood.setForDistance(18.0)  // Close
        }
        
        Gamepads.gamepad1.b whenBecomesTrue {
            Hood.setForDistance(36.0)  // Far
        }
        
        Gamepads.gamepad1.x whenBecomesTrue {
            Hood.setForDistance(30.0)  // Mid
        }
    }
    
    override fun onUpdate() {
        telemetry.addData("=== HOOD TEST ===", "")
        telemetry.addData("D-pad UP", "Open (far)")
        telemetry.addData("D-pad DOWN", "Close (near)")
        telemetry.addData("D-pad LEFT", "Mid")
        telemetry.addData("D-pad RIGHT", "Quarter")
        
        telemetry.addData("--- AUTO MODE ---", "")
        telemetry.addData("A = Distance 18\"", "")
        telemetry.addData("B = Distance 36\"", "")
        telemetry.addData("X = Distance 30\"", "")
        
        telemetry.addData("--- STATUS ---", "")
        telemetry.addData("Position", "%.3f".format(Hood.position))
        telemetry.addData("Target", "%.3f".format(Hood.targetPosition))
        telemetry.addData("Last Distance", "%.1f\"".format(Hood.currentDistance))
    }
}
