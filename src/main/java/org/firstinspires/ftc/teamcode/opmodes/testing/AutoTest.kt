package org.firstinspires.ftc.teamcode.opmodes.testing

import com.pedropathing.follower.Follower
import com.pedropathing.geometry.Pose
import com.pedropathing.pathing.PathBuilder
import com.pedropathing.pathing.Path
import dev.nextftc.core.components.BindingsComponent
import dev.nextftc.core.components.SubsystemComponent
import dev.nextftc.extensions.pedro.PedroComponent
import dev.nextftc.ftc.NextFTCOpMode
import dev.nextftc.ftc.components.BulkReadComponent

/**
 * Autonomous Test
 * Tests basic path following with Pedro
 */
class AutoTest : NextFTCOpMode() {
    
    private lateinit var autoPath: Path
    
    init {
        addComponents(
            PedroComponent(org.firstinspires.ftc.teamcode.util.Constants::createFollower),
            SubsystemComponent(),
            BulkReadComponent,
            BindingsComponent
        )
    }
    
    override fun onInit() {
        // Build a simple test path
        autoPath = PathBuilder()
            .lineTo(Pose(24.0, 0.0))
            .lineTo(Pose(24.0, 24.0))
            .lineTo(Pose(48.0, 24.0))
            .splineTo(Pose(48.0, 0.0))
            .build()
        
        follower.path(autoPath)
    }
    
    override fun onStartButtonPressed() {
        follower.followPath(autoPath, true)
    }
    
    override fun onUpdate() {
        follower.update()
        
        telemetry.addData("=== AUTO TEST ===", "")
        telemetry.addData("State", follower.state.name)
        telemetry.addData("X", "%.1f".format(follower.pose.x))
        telemetry.addData("Y", "%.1f".format(follower.pose.y))
        telemetry.addData("Heading", "%.1f°".format(Math.toDegrees(follower.pose.heading)))
        telemetry.addData("Target X", "%.1f".format(follower.targetPose?.x ?: 0.0))
        telemetry.addData("Target Y", "%.1f".format(follower.targetPose?.y ?: 0.0))
        
        telemetry.addData("=== INSTRUCTIONS ===", "")
        telemetry.addData("Press Start to run path", "")
    }
}
