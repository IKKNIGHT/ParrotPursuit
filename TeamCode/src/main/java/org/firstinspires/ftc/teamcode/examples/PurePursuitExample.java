package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.PathChain;
import org.firstinspires.ftc.teamcode.paths.Point;
import org.firstinspires.ftc.teamcode.paths.PolyLine;

/**
 * Example OpMode demonstrating Pure Pursuit path following
 * This shows how to create paths and use the pure pursuit algorithm
 */
@Autonomous(name = "Pure Pursuit Example", group = "Examples")
public class PurePursuitExample extends LinearOpMode {
    
    private MecanumDrive drive;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Instructions", "This example demonstrates different path following scenarios");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Example 1: Simple straight line path
            runStraightLineExample();
            
            sleep(2000);
            
            // Example 2: Multi-point path
            runMultiPointExample();
            
            sleep(2000);
            
            // Example 3: Complex path chain
            runComplexPathExample();
        }
    }
    
    /**
     * Example 1: Follow a simple straight line
     */
    private void runStraightLineExample() {
        telemetry.addData("Example", "1 - Straight Line Path");
        telemetry.update();
        
        // Create a straight line from (0,0) to (48,0) - 4 feet forward
        LinearPath straightPath = new LinearPath(
            new Point(0, 0),
            new Point(48, 0)
        );
        
        // Start following the path
        drive.followPath(straightPath);
        
        // Keep updating until path is complete
        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.updateLocalizer();
            drive.updatePurePursuit();
            telemetry.addData("Path Progress", drive.getPathProgress() * 100 + "%");
            telemetry.update();
            
            sleep(20); // 50Hz update rate
        }
        
        telemetry.addData("Example 1", "Complete!");
        telemetry.update();
    }
    
    /**
     * Example 2: Follow a multi-point path
     */
    private void runMultiPointExample() {
        telemetry.addData("Example", "2 - Multi-Point Path");
        telemetry.update();
        
        // Create a path that goes through multiple points
        PolyLine multiPath = new PolyLine(
            new Point(48, 0),    // Start where we ended
            new Point(48, 24),   // Go sideways 2 feet
            new Point(24, 24),   // Go back 2 feet
            new Point(24, 48),   // Go forward 2 feet
            new Point(0, 48)     // Go back to start Y position
        );
        
        // Start following the path
        drive.followPath(multiPath);
        
        // Keep updating until path is complete
        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.updateLocalizer();
            drive.updatePurePursuit();
            telemetry.addData("Path Progress", drive.getPathProgress() * 100 + "%");
            telemetry.update();
            
            sleep(20); // 50Hz update rate
        }
        
        telemetry.addData("Example 2", "Complete!");
        telemetry.update();
    }
    
    /**
     * Example 3: Complex path using PathChain
     */
    private void runComplexPathExample() {
        telemetry.addData("Example", "3 - Complex Path Chain");
        telemetry.update();
        
        // Create multiple path segments
        LinearPath segment1 = new LinearPath(
            new Point(0, 48),
            new Point(-24, 24)
        );
        
        LinearPath segment2 = new LinearPath(
            new Point(-24, 24),
            new Point(-48, 0)
        );
        
        LinearPath segment3 = new LinearPath(
            new Point(-48, 0),
            new Point(0, 0)  // Return to start
        );
        
        // Chain the segments together
        PathChain complexPath = new PathChain(segment1, segment2, segment3);
        
        // Adjust lookahead distance for this complex path
        drive.setLookaheadDistance(15.0); // Shorter lookahead for tighter turns
        
        // Start following the path
        drive.followPath(complexPath);
        
        // Keep updating until path is complete
        while (opModeIsActive() && drive.isFollowingPath()) {
            drive.updateLocalizer();
            drive.updatePurePursuit();
            telemetry.addData("Path Progress", drive.getPathProgress() * 100 + "%");
            telemetry.addData("Lookahead Distance", drive.getLookaheadDistance());
            telemetry.update();
            
            sleep(20); // 50Hz update rate
        }
        
        telemetry.addData("Example 3", "Complete!");
        telemetry.addData("All Examples", "Complete! Pure Pursuit is working.");
        telemetry.update();
    }
}