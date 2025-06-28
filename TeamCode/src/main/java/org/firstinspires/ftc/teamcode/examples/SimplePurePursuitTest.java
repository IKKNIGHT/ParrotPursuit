package org.firstinspires.ftc.teamcode.examples;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.MecanumDrive;
import org.firstinspires.ftc.teamcode.geometry.Pose2d;
import org.firstinspires.ftc.teamcode.geometry.Rotation2d;
import org.firstinspires.ftc.teamcode.paths.LinearPath;
import org.firstinspires.ftc.teamcode.paths.Point;

/**
 * Simple test OpMode for Pure Pursuit - just follows a straight line
 * Use this for initial testing and tuning
 */
@Autonomous(name = "Pure Pursuit Simple Test", group = "Testing")
public class SimplePurePursuitTest extends LinearOpMode {
    
    private MecanumDrive drive;
    
    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize the drive system at origin
        Pose2d startPose = new Pose2d(0, 0, new Rotation2d(0));
        drive = new MecanumDrive(hardwareMap, telemetry, startPose);
        
        telemetry.addData("Status", "Initialized");
        telemetry.addData("Test", "Will drive forward 36 inches using Pure Pursuit");
        telemetry.addData("Instructions", "Press play to start");
        telemetry.update();
        
        waitForStart();
        
        if (opModeIsActive()) {
            // Create a simple straight line path - drive forward 3 feet
            LinearPath testPath = new LinearPath(
                new Point(0, 0),     // Start at origin
                new Point(36, 0)     // Drive forward 36 inches
            );
            
            telemetry.addData("Status", "Starting Pure Pursuit");
            telemetry.update();
            
            // Start following the path
            drive.followPath(testPath);
            
            // Main control loop
            while (opModeIsActive() && drive.isFollowingPath()) {
                // Update robot position
                drive.updateLocalizer();
                
                // Update pure pursuit control
                drive.updatePurePursuit();
                
                // Update telemetry (limit to avoid performance issues)
                if (System.currentTimeMillis() % 200 < 20) { // Update every 200ms
                    telemetry.addData("Status", "Following Path");
                    telemetry.addData("Progress", String.format("%.1f%%", drive.getPathProgress() * 100));
                    telemetry.addData("Position", String.format("(%.1f, %.1f)", 
                        drive.getLocalizer().getPoseEstimate().getX(), 
                        drive.getLocalizer().getPoseEstimate().getY()));
                    telemetry.update();
                }
                
                // Run at ~50Hz
                sleep(20);
            }
            
            // Path completed
            telemetry.addData("Status", "Path Complete!");
            telemetry.update();
            
            // Stop the robot
            drive.stopPathFollowing();
        }
    }
}